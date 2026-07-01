// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/editor/widgets/performance/performance_window.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>  // NOLINT: Need to sort by thread id.
#include <utility>

#include "absl/debugging/leak_check.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "implot/implot.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/editor/widgets/performance/config.h"
#include "core/editor/widgets/performance/frame_time_panel.h"
#include "core/editor/widgets/performance/memory_panel.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/editor/widgets/performance/profiler_details_section.h"
#include "core/editor/widgets/performance/render_info_panel.h"
#include "core/ncsb/update_system.h"
#include "core/performance/profiler.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"
#include "core/view/view_events.h"

namespace imp::editor {

namespace {

#if IMP_PLATFORM(ANDROID)
constexpr int kPanelHeight = 50;
#else
constexpr int kPanelHeight = 120;
#endif

// Height of the splitter between the graphs and the details.
constexpr float kSplitterHeight = 2.0f;

// Height of the selection area for the splitter.
constexpr float kSplitterSelectionHeight = 8.0f;

// Color of the splitter between panels.
constexpr ImU32 kSplitterColor = IM_COL32(255, 255, 255, 150);

// Minimum height for the graphs panel when resizing.
constexpr float kGraphsMinHeight = 100.0f;

// Minimum height for the details panel when resizing.
constexpr float kDetailsMinHeight = 300.0f;

// Width value that tells ImGui to make this element fill the remaining space.
constexpr int kFlexibleWidth = -1;

}  // namespace

PerformanceWindow::PerformanceWindow(BaseView& view) : view_(view) {
  {
    // TODO Either fix the leak or find a way to automatically
    // disable the leak check for all usages of ImPlot.
    absl::LeakCheckDisabler disabler;
    ImPlot::CreateContext();
  }
  time_span_seconds_ = details::kDefaultTimeSpanSeconds;
  if (Profiler::IsMemoryGraphSupported()) {
    AddPanel(std::make_unique<MemoryPanel>(
        *this,
        details::kNumDisplayValuesPerSecond * details::kMaxTimeSpanSeconds));
  }
  AddPanel(std::make_unique<RenderInfoPanel>(
      *this, view,
      details::kNumDisplayValuesPerSecond * details::kMaxTimeSpanSeconds));
  AddPanel(std::make_unique<FrameTimePanel>(
      *this, view,
      details::kNumDisplayValuesPerSecond * details::kMaxTimeSpanSeconds));

  selected_sample_thread_id_ = Profiler::GetMainThreadId();
  details_panel_ = std::make_unique<ProfilerDetailsSection>(*this);

  post_frame_connection_ = view.GetDispatcher().Connect(
      [this](const ViewPostRenderEvent&) { OnViewPostRender(); });
}

PerformanceWindow::~PerformanceWindow() = default;

void PerformanceWindow::AddPanel(std::unique_ptr<MonitorPanel> monitor_panel) {
  monitor_panels_.push_back(std::move(monitor_panel));
}

void PerformanceWindow::DrawImGui() { DrawMonitorPanels(); }

// TODO refactor each monitor panel into its own class for custom
// plotting now that we're using ImPlot
void PerformanceWindow::DrawMonitorPanels() {
  IMP_TRACE();
  const MonitorState monitor_state = GetMonitorState();
  bool clicked = ImGui::Button(
      monitor_state == MonitorState::kPaused ? "Record" : "Pause");
  if (clicked) {
    MonitorState new_state = monitor_state == MonitorState::kPaused
                                 ? MonitorState::kRunning
                                 : MonitorState::kPaused;
    SetMonitorState(new_state);
  }

  const int num_panels = monitor_panels_.size();
  if (num_panels == 0) return;

  const float full_graphs_height =
      static_cast<float>(num_panels) * kPanelHeight +
      ImGui::GetStyle().ItemSpacing.y * static_cast<float>(num_panels - 1);

  // If the graphs height is not set, set it to whatever is needed to display
  // all graphs with no clipping.
  if (graphs_height_ <= 0.0f) {
    graphs_height_ = full_graphs_height;
  }

  // Keeps the height of the graphs section from growing larger than is needed
  // to display all graphs.
  graphs_height_ = std::min(graphs_height_, full_graphs_height);

  // Calculate the available height for the graphs and details sections.
  // We want the details section to fill the remaining space in the window,
  // but stay at least kDetailsMinHeight.
  const float available_height = ImGui::GetContentRegionAvail().y;
  details_height_ =
      std::max(kDetailsMinHeight,
               available_height - graphs_height_ - kSplitterSelectionHeight);

  ImGui::BeginChild("##graphs_section", ImVec2(kFlexibleWidth, graphs_height_),
                    ImGuiChildFlags_None);
  for (auto& monitor_panel : monitor_panels_) {
    monitor_panel->DrawPanel(-1, kPanelHeight, time_span_seconds_);
  }
  ImGui::EndChild();

  DrawSplitter();

  ImGui::BeginChild("##details_section",
                    ImVec2(kFlexibleWidth, details_height_));

  // The details section is currently only used by the CPU/Frame profiling.
  details_panel_->Draw();

  ImGui::EndChild();
}

void PerformanceWindow::DrawSplitter() {
  ImGui::BeginChild("##splitter_child",
                    ImVec2(kFlexibleWidth, kSplitterSelectionHeight));

  const ImVec2 child_pos = ImGui::GetCursorScreenPos();
  const float child_width = ImGui::GetContentRegionAvail().x;

  // Make the splitter selection area taller than the visible rectangle.
  ImVec2 rect_min = child_pos;
  rect_min.y += kSplitterSelectionHeight / 2.0f - kSplitterHeight / 2.0f;
  const ImVec2 rect_max =
      ImVec2(child_pos.x + child_width, rect_min.y + kSplitterHeight);

  ImGui::GetWindowDrawList()->AddRectFilled(rect_min, rect_max, kSplitterColor);

  ImGui::SplitterBehavior(
      ImRect(child_pos, ImVec2(child_pos.x + child_width,
                               child_pos.y + kSplitterSelectionHeight)),
      ImGui::GetID("##performance_splitter"), ImGuiAxis_Y, &graphs_height_,
      &details_height_, kGraphsMinHeight, kDetailsMinHeight, 0.0f);
  ImGui::EndChild();
}

void PerformanceWindow::OnViewPostRender() {
  if (GetMonitorState() == MonitorState::kPaused) {
    return;
  }

  const int64_t frame_index = Profiler::GetCurrentFrameIndex() - 1;
  sample_processor_.ProcessMainThreadSamples(frame_index);
  samples_processed_since_last_update_ = true;

  FrameTime frame_time = view_.GetFrameTime();
  for (auto& monitor_panel : monitor_panels_) {
    monitor_panel->Update(frame_time.GetElapsedTime(),
                          frame_time.GetDeltaTime());
  }
}

void PerformanceWindow::SelectFrame(int frame_number) {
  SetMonitorState(MonitorState::kPaused);
  selected_frame_start_ = frame_number;
  selected_frame_end_ = frame_number;
}

void PerformanceWindow::SetSelectedSampleName(absl::string_view name) {
  selected_sample_name_ = std::string(name);
  selected_sample_changed_ = true;
}

void PerformanceWindow::SetSelectedSampleThreadId(std::thread::id thread_id) {
  selected_sample_thread_id_ = thread_id;
  selected_sample_changed_ = true;
}

void PerformanceWindow::SelectFrames(int start_frame, int end_frame) {
  SetMonitorState(MonitorState::kPaused);
  if (start_frame > end_frame) {
    selected_frame_start_ = end_frame;
    selected_frame_end_ = start_frame;
  } else {
    selected_frame_start_ = start_frame;
    selected_frame_end_ = end_frame;
  }
}

void PerformanceWindow::SetMonitorState(MonitorState monitor_state) {
  monitor_state_ = monitor_state;
  Profiler::SetPaused(monitor_state == MonitorState::kPaused);
}

}  // namespace imp::editor
