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
#include <cstdint>
#include <string>
#include <thread>  // NOLINT: Need to use std::thread::id.
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/debugging/symbolize.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/profiler_data_provider.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/editor/widgets/performance/search_filter.h"
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

// Width of the column in collapsed mode that displays the count of call stacks.
constexpr float kCountColumnWidth = 50.0f;

struct CallstackKey {
  const imp::MemoryStats::Callstack* callstack;

  bool operator==(const CallstackKey& other) const {
    if (callstack->depth != other.callstack->depth) return false;

    for (uint32_t k = 0; k < callstack->depth; ++k) {
      if (callstack->callstack[k] == other.callstack->callstack[k]) continue;

      return false;
    }

    return true;
  }

  template <typename H>
  friend H AbslHashValue(H h, const CallstackKey& key) {
    // Absl hashing function optimized for an array of value types.
    // Hashes all the pointers in the call stack from start to call stack depth.
    return H::combine_contiguous(std::move(h), key.callstack->callstack.data(),
                                 key.callstack->depth);
  }
};
}  // namespace

namespace imp::editor {

absl::StatusOr<absl::Span<SampleNode* const>> CallstackPanel::GetSamples(
    const int frame_index, SampleProcessor& sample_processor,
    const std::thread::id thread_id,
    const absl::string_view selected_sample_name) {
  if (selected_sample_name.empty()) {
    return absl::InvalidArgumentError(
        "Select a sample to view any recorded call stack data.");
  }

  if (thread_id == Profiler::GetMainThreadId()) {
    const ProcessedSamples& processed_frame =
        sample_processor.GetProcessedFrame(frame_index);

    absl::Span<SampleNode* const> samples =
        processed_frame.GetSamplesByName(selected_sample_name);

    if (samples.empty()) {
      return absl::NotFoundError("Sample not found on this thread.");
    }

    return samples;
  }

  // TODO (broken link)(robinsonjordan): Callstack UI for worker threads.
  return absl::UnimplementedError(
      "Callstack data is not supported for worker threads.");
}

void CallstackPanel::DrawPanel(const float width,
                               ProfilerDataProvider& data_provider,
                               const std::thread::id thread_id,
                               const absl::string_view search_query,
                               const bool collapse_callstacks) {
  IMP_TRACE();

  if (collapse_callstacks != collapse_callstacks_) {
    collapse_callstacks_ = collapse_callstacks;
    rebuild_table_entries_ = true;
    selected_sample_node_index_ = kInvalidId;
    selected_start_index_ = kInvalidId;
    selected_end_index_ = kInvalidId;
  }

  const int end_frame = data_provider.GetSelectedFrameEnd();
  SampleProcessor& sample_processor = data_provider.GetSampleProcessor();
  const absl::string_view selected_sample_name =
      data_provider.GetSelectedSampleName();

  // Reset the selected sample if we change the selected frame or sample.
  if (end_frame != last_frame_index_ ||
      selected_sample_name != last_selected_sample_name_) {
    selected_sample_node_index_ = kInvalidId;
    selected_start_index_ = kInvalidId;
    selected_end_index_ = kInvalidId;
    last_frame_index_ = end_frame;
    last_selected_sample_name_ = selected_sample_name;
    rebuild_table_entries_ = true;
  }

  if (search_query != last_search_query_) {
    last_search_query_ = std::string(search_query);
    rebuild_table_entries_ = true;
  }

  // Dynamic height based on available space with a minimum.
  const float available_height = ImGui::GetContentRegionAvail().y;
  const float child_height = std::max(kMinPanelHeight, available_height);

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::BeginChild("##callstackpanel", ImVec2(width, child_height),
                    ImGuiChildFlags_Borders);

  absl::StatusOr<absl::Span<SampleNode* const>> samples =
      GetSamples(end_frame, sample_processor, thread_id, selected_sample_name);

  if (samples.ok()) {
    // If there are samples for this frame, draw the callstack table.
    DrawCallstackPanel(*samples, search_query);
  } else {
    // Otherwise, instruct the user on what to do to see callstack data.
    ImGui::Text("%s", samples.status().message().data());
  }

  ImGui::EndChild();
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding
}

void CallstackPanel::DrawCallstackPanel(absl::Span<SampleNode* const> samples,
                                        const absl::string_view search_query) {
  if (upper_panel_height_ <= 0.0f) {
    upper_panel_height_ =
        ImGui::GetContentRegionAvail().y - kLowerPanelStartingHeight - 20.0f;
  }

  // Draw the table showing the last function in each callstack and bytes alloc.
  DrawCallstackTable(samples, search_query);

  // Draw a splitter between the table and the full callstack panel.
  DrawSplitter();

  // Draw the detailed callstack lower panel.
  DrawFullCallstackReadout();
}

bool CallstackPanel::AnyStackFrameMatchesSearch(
    const MemoryStats::Callstack& callstack,
    const absl::string_view search_query) {
  if (search_query.empty()) return true;

  for (int k = 0; k < callstack.depth; ++k) {
    const void* addr = callstack.callstack[k];
    const std::string& func_name = GetCallstackSymbol(addr);
    if (MatchesSearchQuery(func_name, search_query)) return true;
  }
  return false;
}

void CallstackPanel::RebuildTableEntries(absl::Span<SampleNode* const> samples,
                                         const absl::string_view search_query) {
  callstack_table_entries_.clear();
  const size_t sample_count = samples.size();
  has_stale_callstacks_ = false;

  static absl::flat_hash_map<CallstackKey, size_t> unique_callstacks;
  unique_callstacks.clear();

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
      has_stale_callstacks_ = true;
      break;
    }

    const std::thread::id thread_id = result->GetThreadId();

    for (int j = start_index; j < end_index; ++j) {
      const MemoryStats::Callstack& callstack =
          MemoryStats::Get().GetCallstack(j);

      // If the callstack is empty or from a different thread, skip it.
      if (callstack.depth == 0 || callstack.thread_id != thread_id) continue;

      // Filter out any callstacks that don't match the search query.
      // Checks the entire callstack for any matching frames.
      if (!AnyStackFrameMatchesSearch(callstack, search_query)) continue;

      if (collapse_callstacks_) {
        const CallstackKey key{&callstack};
        auto [it, inserted] =
            unique_callstacks.try_emplace(key, callstack_table_entries_.size());

        if (!inserted) {
          callstack_table_entries_[it->second].count++;
          callstack_table_entries_[it->second].total_size += callstack.size;
          continue;
        }
      }

      callstack_table_entries_.push_back(
          {i, j, 1, static_cast<uint32_t>(callstack.size)});
    }
  }
}

void CallstackPanel::DrawCallstackTable(absl::Span<SampleNode* const> samples,
                                        const absl::string_view search_query) {
  IMP_TRACE();
  ImGui::BeginChild("##callstacktable", ImVec2(-1, upper_panel_height_));

  if (rebuild_table_entries_) {
    RebuildTableEntries(samples, search_query);
    rebuild_table_entries_ = false;
  }

  const int column_count = collapse_callstacks_ ? 3 : 2;
  ImGui::PushID(collapse_callstacks_ ? 1 : 0);
  if (ImGui::BeginTable("callstacks", column_count,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("Function");
    if (collapse_callstacks_) {
      ImGui::TableSetupColumn("Count", ImGuiTableColumnFlags_WidthFixed,
                              kCountColumnWidth);
    }
    ImGui::TableSetupColumn("Memory", ImGuiTableColumnFlags_WidthFixed,
                            kMemoryColumnWidth);
    ImGui::TableHeadersRow();

    if (has_stale_callstacks_) {
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

        if (collapse_callstacks_) {
          ImGui::TableSetColumnIndex(1);
          ImGui::Text("%d", entry.count);
          ImGui::TableSetColumnIndex(2);
        } else {
          ImGui::TableSetColumnIndex(1);
        }
        ImGui::Text("%u bytes", entry.total_size);
      }
    }
    ImGui::EndTable();
  }
  ImGui::PopID();
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

  int count = 1;
  uint32_t display_size = callstack.size;
  for (const TableEntry& entry : callstack_table_entries_) {
    if (entry.sample_index == selected_sample_node_index_ &&
        entry.callstack_index == selected_start_index_) {
      count = entry.count;
      display_size = entry.total_size;
      break;
    }
  }

  std::string full_callstack_text;
  if (count > 1) {
    full_callstack_text = absl::StrFormat(
        "Allocations: %d times (total %u bytes):\n", count, display_size);
  } else {
    full_callstack_text = absl::StrFormat(
        "Allocation %d (%u bytes):\n", selected_start_index_, callstack.size);
  }

  // Iterate over all the function pointers in the call stack and create a
  // string representation of it with symbolized function names if available.
  for (int j = 0; j < callstack.depth; ++j) {
    const void* addr = callstack.callstack[j];
    const std::string func_name = GetCallstackSymbol(addr);
    full_callstack_text += absl::StrFormat("  [%d] %s\n", j, func_name);
  }

  ImGui::PushStyleColor(ImGuiCol_FrameBg,
                        ImGui::GetStyleColorVec4(ImGuiCol_ChildBg));
  // +1 for the null terminator or this fails an assert and crashes if you try
  // to highlight and copy the text.
  const size_t text_size = full_callstack_text.size() + 1;
  ImGui::InputTextMultiline("##fullcallstack", &full_callstack_text[0],
                            text_size, ImVec2(-1, -1),
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
