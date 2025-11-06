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

#include <memory>
#include <utility>

#include "absl/debugging/leak_check.h"
#include "dear_imgui/imgui.h"
#include "implot/implot.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/editor/widgets/performance/config.h"
#include "core/editor/widgets/performance/frame_time_panel.h"
#include "core/editor/widgets/performance/monitor_panel.h"
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
constexpr int kPanelHeight = 200;
#endif

using MonitorState = MonitorPanel::MonitorState;

}  // namespace

PerformanceWindow::PerformanceWindow(BaseView& view) : view_(view) {
  {
    // TODO Either fix the leak or find a way to automatically
    // disable the leak check for all usages of ImPlot.
    absl::LeakCheckDisabler disabler;
    ImPlot::CreateContext();
  }
  time_span_seconds_ = details::kDefaultTimeSpanSeconds;
  AddPanel(std::make_unique<RenderInfoPanel>(
      view,
      details::kNumDisplayValuesPerSecond * details::kMaxTimeSpanSeconds));
  AddPanel(std::make_unique<FrameTimePanel>(
      view,
      details::kNumDisplayValuesPerSecond * details::kMaxTimeSpanSeconds));

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
  IMP_TRACE_NAME("PerformanceWindow::DrawMonitorPanels");
  ImGui::SliderFloat("Time span", &time_span_seconds_, 1,
                     details::kMaxTimeSpanSeconds, "%.1f s");
  bool clicked = ImGui::Button(
      monitor_state_ == MonitorState::kPaused ? "Resume" : "Pause");
  if (clicked) {
    monitor_state_ = monitor_state_ == MonitorState::kPaused
                         ? MonitorState::kRunning
                         : MonitorState::kPaused;
    Profiler::SetPaused(monitor_state_ == MonitorState::kPaused ? true : false);

    for (auto& monitor_panel : monitor_panels_) {
      monitor_panel->OnStateChanged(monitor_state_);
    }
  }

  for (auto& monitor_panel : monitor_panels_) {
    monitor_panel->DrawPanel(-1, kPanelHeight, time_span_seconds_);
  }
}

void PerformanceWindow::OnViewPostRender() {
  if (monitor_state_ == MonitorState::kPaused) {
    return;
  }
  FrameTime frame_time = view_.GetFrameTime();
  for (auto& monitor_panel : monitor_panels_) {
    monitor_panel->Update(frame_time.GetElapsedTime(),
                          frame_time.GetDeltaTime());
  }
}

}  // namespace imp::editor
