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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_CALLSTACK_PANEL_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_CALLSTACK_PANEL_H_

#include <string>
#include <thread>  // NOLINT: Need to use std::thread::id.
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"

namespace imp::editor {

class FrameTimePanel;

// Displays memory allocations and call stacks of where they originated.
// Rendered as a subpanel of the FrameTimePanel.
class CallstackPanel {
 public:
  CallstackPanel() {}
  ~CallstackPanel() = default;

  void DrawPanel(float width, int frame_index, FrameTimePanel& frame_time_panel,
                 SampleProcessor& sample_processor, std::thread::id thread_id);

 private:
  struct TableEntry {
    int sample_index;
    int callstack_index;
  };

  // Initial height of the lower panel containing the full call stack readout.
  static constexpr float kLowerPanelStartingHeight = 150.0f;

  // Id to use when nothing is selected.
  static constexpr int kInvalidId = -1;

  // Returns all samples with a specific name for a given frame and thread.
  absl::StatusOr<std::vector<SampleNode*>*> GetSamples(
      int frame_index, SampleProcessor& sample_processor,
      std::thread::id thread_id, absl::string_view selected_sample_name);

  // Draws the call stack table, splitter, and the full call stack panel.
  void DrawCallstackPanel(std::vector<SampleNode*>& samples);

  // Draws the table contents with a row for each allocation in the samples.
  void DrawCallstackTable(std::vector<SampleNode*>& samples);

  // Draws the splitter between the table and the full call stack panel.
  void DrawSplitter();

  // Draws the full call stack panel containing the call stack readout.
  void DrawFullCallstackReadout();

  // Returns the symbol name for a given address. If the symbol cannot be
  // resolved, the address is returned as a hex string.
  std::string& GetCallstackSymbol(const void* addr);

  int selected_sample_node_index_ = kInvalidId;
  int selected_start_index_ = kInvalidId;
  int selected_end_index_ = kInvalidId;
  int last_frame_index_ = kInvalidId;
  absl::string_view last_selected_sample_name_;
  float upper_panel_height_ = -1.0f;
  float lower_panel_height_ = kLowerPanelStartingHeight;
  // Cache of symbolized addresses to avoid repeated calls to absl::Symbolize.
  // May end up being a few KB in size with heavy use of the call stack panel.
  absl::flat_hash_map<const void*, std::string> callstack_cache_;
  std::vector<TableEntry> callstack_table_entries_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_CALLSTACK_PANEL_H_
