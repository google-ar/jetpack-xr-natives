// Copyright 2026 Google LLC
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

#include "core/editor/widgets/performance/profiler_details_section.h"

#include <string>
#include <thread>  // NOLINT: Need to sort by thread id.
#include <vector>

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "dear_imgui/misc/cpp/imgui_stdlib.h"
#include "core/editor/widgets/performance/profiler_data_provider.h"
#include "core/performance/memory_stats.h"
#include "core/performance/profiler.h"

namespace imp::editor {

namespace {

// Width of the option bar items (record call stacks, show call stacks, etc).
constexpr float kOptionBarItemWidth = 200.0f;

// Width of the splitter between the sample view and the call stack view.
constexpr float kSplitterWidth = 2.0f;

// Width of the selection area for the splitter.
constexpr float kSplitterSelectionWidth = 8.0f;

// Minimum width for the call stack panel.
constexpr float kCallstackPanelMinWidth = 200.0f;

// Minimum width for the sample view.
constexpr float kSampleViewMinWidth = 500.0f;

// Color of the splitter between panels.
constexpr ImU32 kSplitterColor = IM_COL32(255, 255, 255, 150);

}  // namespace

ProfilerDetailsSection::ProfilerDetailsSection(
    ProfilerDataProvider& data_provider)
    : data_provider_(data_provider) {}

void ProfilerDetailsSection::Draw() {
  if (!thread_set_) {
    data_provider_.SetSelectedSampleThreadId(Profiler::GetCachedThreadId());
    thread_set_ = true;
  }

  // Options for changing the view and thread to display samples for.
  DrawOptionsBar();

  float content_width = ImGui::GetContentRegionAvail().x;
  if (show_callstack_) {
    if (sample_view_width_ < 0) {
      sample_view_width_ = ImGui::GetContentRegionAvail().x -
                           callstack_panel_width_ - kSplitterSelectionWidth -
                           ImGui::GetStyle().ItemSpacing.x * 2;
    }
    content_width = sample_view_width_;
  }

  const std::thread::id thread_id = data_provider_.GetSelectedSampleThreadId();

  if (view_mode_ == ViewMode::kHierarchy) {
    hierarchy_panel_.DrawPanel(content_width, data_provider_, thread_id,
                               search_query_);
  } else {
    flame_graph_.DrawPanel(content_width, data_provider_, search_query_);
  }

  if (show_callstack_) {
    ImGui::SameLine();
    DrawSplitter();
    ImGui::SameLine();

    callstack_panel_.DrawPanel(ImGui::GetContentRegionAvail().x, data_provider_,
                               thread_id, search_query_, show_callstack_ == 2);
  }
}

void ProfilerDetailsSection::DrawOptionsBar() {
  // Toggle to switch between Hierarchy and Flame Graph.
  if (ImGui::RadioButton("Hierarchy", view_mode_ == ViewMode::kHierarchy)) {
    view_mode_ = ViewMode::kHierarchy;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Flame Graph", view_mode_ == ViewMode::kFlameGraph)) {
    view_mode_ = ViewMode::kFlameGraph;
  }

  if (view_mode_ == ViewMode::kHierarchy) {
    ImGui::SameLine();
    ImGui::PushItemWidth(kOptionBarItemWidth);
    DrawThreadSelector();
    ImGui::PopItemWidth();
  }

  ImGui::SameLine();
  ImGui::PushItemWidth(kOptionBarItemWidth);
  ImGui::InputTextWithHint("##search", "Search...", &search_query_);
  ImGui::PopItemWidth();

  // Memory call stacks are not supported on all platforms.
  if (!MemoryStats::IsCallstackTrackingSupported()) return;

  static const char* kCallstackOptions[]{"Hide Callstacks", "Show Callstacks",
                                         "Show Collapsed"};

  // Calculate the width of the items to be right-aligned.
  // Combo width + Checkbox width + Item spacing.
  const float checkbox_width = ImGui::GetFrameHeight() +
                               ImGui::GetStyle().ItemInnerSpacing.x +
                               ImGui::CalcTextSize("Memory Callstacks").x;
  const float right_side_width =
      kOptionBarItemWidth + checkbox_width + ImGui::GetStyle().ItemSpacing.x;

  ImGui::SameLine();
  // Take up all the space available in the center to right-align the memory
  // call stacks options.
  const float dummy_width = ImGui::GetContentRegionAvail().x - right_side_width;
  if (dummy_width > 0.0f) {
    ImGui::Dummy(ImVec2(dummy_width, 0));
    ImGui::SameLine();
  }

  ImGui::PushItemWidth(kOptionBarItemWidth);
  ImGui::Combo("##callstack", &show_callstack_, kCallstackOptions, 3);

  ImGui::SameLine();
  bool record_callstacks = Profiler::IsRecordingCallstacks();
  if (ImGui::Checkbox("Memory Callstacks", &record_callstacks)) {
    Profiler::SetRecordingCallstacks(record_callstacks);
  }
  ImGui::PopItemWidth();
}

void ProfilerDetailsSection::DrawThreadSelector() {
  const std::thread::id selected_id =
      data_provider_.GetSelectedSampleThreadId();
  const std::string selected_name =
      std::string(Profiler::GetThreadName(selected_id));

  // Allow us to switch between threads and see their samples.
  if (!ImGui::BeginCombo("##combo", selected_name.c_str())) return;

  const std::vector<std::thread::id> thread_ids = Profiler::GetThreadIds();

  for (int i = 0; i < thread_ids.size(); ++i) {
    const std::thread::id thread_id = thread_ids[i];
    absl::string_view it_thread_name = Profiler::GetThreadName(thread_id);
    const bool is_selected = (selected_id == thread_id);

    if (ImGui::Selectable(it_thread_name.data(), is_selected)) {
      data_provider_.SetSelectedSampleThreadId(thread_id);
    }

    if (is_selected) {
      ImGui::SetItemDefaultFocus();
    }
  }

  ImGui::EndCombo();
}

void ProfilerDetailsSection::DrawSplitter() {
  ImGui::BeginChild("##splitter_callstack",
                    ImVec2(kSplitterSelectionWidth, -1));

  const ImVec2 child_pos = ImGui::GetCursorScreenPos();
  const float child_height = ImGui::GetContentRegionAvail().y;

  // Make the splitter selection area taller than the visible rectangle.
  ImVec2 rect_min = child_pos;
  rect_min.x += kSplitterSelectionWidth / 2.0f - kSplitterWidth / 2.0f;
  const ImVec2 rect_max =
      ImVec2(rect_min.x + kSplitterWidth, child_pos.y + child_height);
  ImGui::GetWindowDrawList()->AddRectFilled(rect_min, rect_max,
                                            ImGui::GetColorU32(kSplitterColor));

  ImGui::SplitterBehavior(
      ImRect(child_pos, ImVec2(child_pos.x + kSplitterSelectionWidth,
                               child_pos.y + child_height)),
      ImGui::GetID("##splitter"), ImGuiAxis_X, &sample_view_width_,
      &callstack_panel_width_, kSampleViewMinWidth, kCallstackPanelMinWidth,
      0.0f);
  ImGui::EndChild();
}

}  // namespace imp::editor
