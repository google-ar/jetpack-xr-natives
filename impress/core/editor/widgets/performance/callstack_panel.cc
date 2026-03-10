/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "core/editor/widgets/performance/callstack_panel.h"

#include <algorithm>
#include <cstddef>
#include <string>
#include <thread>  // NOLINT: Need to use std::thread::id.
#include <vector>

#include "absl/debugging/symbolize.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/frame_time_panel.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/performance/memory_stats.h"
#include "core/performance/profiler.h"
#include "core/performance/profiler_structs.h"

namespace {
// Height of the splitter between the table and the full call stack readout.
constexpr float kSplitterHeight = 2.0f;

// Height of the selection area for the splitter.
constexpr float kSplitterSelectionHeight = 8.0f;

// Color of the splitter between panels.
constexpr ImU32 kSplitterColor = IM_COL32(255, 255, 255, 150);

// Minimum height for the panel no matter how small the window is.
constexpr float kMinPanelHeight = 250.0f;

// Width of the column that displays memory usage.
constexpr float kMemoryColumnWidth = 100.0f;

// Min height for the upper panel when resizing with the splitter.
constexpr float kUpperPanelMinHeight = 100.0f;

// Min height for the lower panel when resizing with the splitter.
constexpr float kLowerPanelMinHeight = 100.0f;
}  // namespace

namespace imp::editor {

absl::StatusOr<std::vector<SampleNode*>*> CallstackPanel::GetSamples(
    int frame_index, SampleProcessor& sample_processor,
    std::thread::id thread_id, absl::string_view selected_sample_name) {
  if (selected_sample_name.empty()) {
    return absl::InvalidArgumentError(
        "Select a sample to view any recorded call stack data.");
  }

  if (thread_id == Profiler::GetMainThreadId()) {
    ProcessedSamples& processed_frame =
        sample_processor.GetProcessedFrame(frame_index);

    const auto& sample_it =
        processed_frame.samples_by_name.find(selected_sample_name);

    if (sample_it == processed_frame.samples_by_name.end()) {
      return absl::NotFoundError("Sample not found on this thread.");
    }

    return &sample_it->second;
  }

  // TODO (broken link)(robinsonjordan): Callstack UI for worker threads.
  return absl::UnimplementedError(
      "Callstack data is not supported for worker threads.");
}

void CallstackPanel::DrawPanel(float width, int frame_index,
                               FrameTimePanel& frame_time_panel,
                               SampleProcessor& sample_processor,
                               std::thread::id thread_id) {
  IMP_TRACE();
  const absl::string_view selected_sample_name =
      frame_time_panel.GetSelectedSampleName();

  // Reset the selected sample if we change the selected frame or sample.
  if (frame_index != last_frame_index_ ||
      selected_sample_name != last_selected_sample_name_) {
    selected_sample_node_index_ = kInvalidId;
    selected_start_index_ = kInvalidId;
    selected_end_index_ = kInvalidId;
    last_frame_index_ = frame_index;
    last_selected_sample_name_ = selected_sample_name;
  }

  // Dynamic height based on available space with a minimum.
  const float available_height = ImGui::GetContentRegionAvail().y;
  const float child_height = std::max(kMinPanelHeight, available_height);

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::BeginChild("##callstackpanel", ImVec2(width, child_height),
                    ImGuiChildFlags_Borders);

  absl::StatusOr<std::vector<SampleNode*>*> samples = GetSamples(
      frame_index, sample_processor, thread_id, selected_sample_name);

  if (samples.ok()) {
    // If there are samples for this frame, draw the callstack table.
    DrawCallstackPanel(**samples);
  } else {
    // Otherwise, instruct the user on what to do to see callstack data.
    ImGui::Text("%s", samples.status().message().data());
  }

  ImGui::EndChild();
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding
}

void CallstackPanel::DrawCallstackPanel(std::vector<SampleNode*>& samples) {
  if (upper_panel_height_ <= 0.0f) {
    upper_panel_height_ =
        ImGui::GetContentRegionAvail().y - kLowerPanelStartingHeight - 20.0f;
  }

  // Draw the table showing the last function in each callstack and bytes alloc.
  DrawCallstackTable(samples);

  // Draw a splitter between the table and the full callstack panel.
  DrawSplitter();

  // Draw the detailed callstack lower panel.
  DrawFullCallstackReadout();
}

void CallstackPanel::DrawCallstackTable(std::vector<SampleNode*>& samples) {
  IMP_TRACE();
  ImGui::BeginChild("##callstacktable", ImVec2(-1, upper_panel_height_));

  callstack_table_entries_.clear();
  const size_t sample_count = samples.size();
  bool has_stale_callstacks = false;

  for (int i = 0; i < sample_count; ++i) {
    const ProfileResult* result = samples[i]->result;
    const int start_index = result->GetCallstackStartIndex();
    const int end_index = result->GetCallstackEndIndex();
    if (start_index == kInvalidId) continue;
    if (end_index == kInvalidId) continue;

    // Check if this sample contains stale callstacks. If so, skip them.
    const size_t callstack_index = MemoryStats::Get().GetCallstackIndex();
    if (callstack_index > MemoryStats::kMaxCallstacks &&
        static_cast<size_t>(start_index) <
            callstack_index - MemoryStats::kMaxCallstacks) {
      has_stale_callstacks = true;
      break;
    }

    const std::thread::id thread_id = result->GetThreadId();

    for (int j = start_index; j < end_index; ++j) {
      const MemoryStats::Callstack& callstack =
          MemoryStats::Get().GetCallstack(j);

      // If the callstack is empty or from a different thread, skip it.
      if (callstack.depth == 0 || callstack.thread_id != thread_id) continue;

      callstack_table_entries_.push_back({i, j});
    }
  }

  if (ImGui::BeginTable("callstacks", 2,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("Function");
    ImGui::TableSetupColumn("Memory", ImGuiTableColumnFlags_WidthFixed,
                            kMemoryColumnWidth);
    ImGui::TableHeadersRow();

    if (has_stale_callstacks) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::Text("Callstacks stale for this sample.");
    }

    // Use ImGuiListClipper to avoid computing all list items at once.
    // This allows us to only run this code block for the visible items.
    // Even if you have 10k call stacks to display, it will only run for the
    // 10-20 visible items. With this optimization we can afford to symbolize
    // all the function addresses in the call stacks.
    ImGuiListClipper clipper;
    clipper.Begin(callstack_table_entries_.size());

    while (clipper.Step()) {
      for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; row++) {
        const TableEntry& entry = callstack_table_entries_[row];
        const MemoryStats::Callstack& callstack =
            MemoryStats::Get().GetCallstack(entry.callstack_index);

        // Get the last function in the callstack to display as the title.
        const void* last_ptr = callstack.callstack[0];
        const std::string& func_name = GetCallstackSymbol(last_ptr);

        const bool is_selected =
            (selected_sample_node_index_ == entry.sample_index &&
             selected_start_index_ == entry.callstack_index);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);

        // Need to push/pop unique ids to avoid duplicate selection when there
        // are multiple samples with the same function name.
        ImGui::PushID(entry.sample_index);
        ImGui::PushID(entry.callstack_index);
        if (ImGui::Selectable(func_name.c_str(), is_selected)) {
          selected_sample_node_index_ = entry.sample_index;
          selected_start_index_ = entry.callstack_index;
          selected_end_index_ = entry.callstack_index;
        }
        ImGui::PopID();
        ImGui::PopID();

        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%u bytes", callstack.size);
      }
    }
    ImGui::EndTable();
  }
  ImGui::EndChild();
}

void CallstackPanel::DrawSplitter() {
  ImGui::BeginChild("##splitter_child", ImVec2(-1, kSplitterSelectionHeight));

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
      ImGui::GetID("##splitter"), ImGuiAxis_Y, &upper_panel_height_,
      &lower_panel_height_, kUpperPanelMinHeight, kLowerPanelMinHeight, 0.0f);
  ImGui::EndChild();
}

void CallstackPanel::DrawFullCallstackReadout() {
  IMP_TRACE();
  ImGui::BeginChild("##fullcallstackpanel",
                    ImVec2(-1, ImGui::GetContentRegionAvail().y));

  if (selected_start_index_ == kInvalidId) {
    ImGui::Text("Select a row in the table to see the full callstack.");
    ImGui::EndChild();
    return;
  }

  const MemoryStats::Callstack& callstack =
      MemoryStats::Get().GetCallstack(selected_start_index_);

  if (callstack.depth <= 0) {
    ImGui::Text("No callstack information available for this range.");
    ImGui::EndChild();
    return;
  }

  std::string full_callstack_text = absl::StrFormat(
      "Allocation %d (%u bytes):\n", selected_start_index_, callstack.size);

  // Iterate over all the function pointers in the call stack and create a
  // string representation of it with symbolized function names if available.
  for (int j = 0; j < callstack.depth; ++j) {
    const void* addr = callstack.callstack[j];
    const std::string func_name = GetCallstackSymbol(addr);
    full_callstack_text += absl::StrFormat("  [%d] %s\n", j, func_name);
  }

  ImGui::PushStyleColor(ImGuiCol_FrameBg,
                        ImGui::GetStyleColorVec4(ImGuiCol_ChildBg));
  ImGui::InputTextMultiline("##fullcallstack", &full_callstack_text[0],
                            full_callstack_text.size(), ImVec2(-1, -1),
                            ImGuiInputTextFlags_ReadOnly);
  ImGui::PopStyleColor();

  ImGui::EndChild();
}

std::string& CallstackPanel::GetCallstackSymbol(const void* addr) {
  const auto it = callstack_cache_.find(addr);

  if (it != callstack_cache_.end()) return it->second;

  constexpr size_t kSymbolBufferSize = 1024;
  char tmp[kSymbolBufferSize];
  std::string& symbol = callstack_cache_[addr];

  // If we can symbolize the address, do so. Otherwise, show the raw ptr.
  if (absl::Symbolize(addr, tmp, sizeof(tmp))) {
    symbol = tmp;
  } else {
    symbol = absl::StrFormat("%p", addr);
  }

  return symbol;
}
}  // namespace imp::editor
