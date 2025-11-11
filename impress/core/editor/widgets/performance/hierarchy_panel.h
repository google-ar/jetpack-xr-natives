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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_CPU_HIERARCHY_PANEL_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_CPU_HIERARCHY_PANEL_H_

#include <cstdint>
#include <thread>  // NOLINT: Need to sort things by thread id.

#include "absl/strings/string_view.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/performance/profiler.h"

namespace imp::editor {

// Panel containing a tree view of performance profiling data.
// Each node in the tree represents a sample taken across a scope.
class HierarchyPanel {
 public:
  HierarchyPanel();
  ~HierarchyPanel();

  void DrawPanel(int frame_index, SampleProcessor& sample_processor);

  absl::string_view GetSelectedSampleName();

 private:
  constexpr static int kMaxTreeDepth = 30;
  // The name of the currently selected sample in the hierarchy.
  absl::string_view selected_sample_name_ = "";
  // The total duration of the root node for the current tree.
  // This is used to calculate the percentage of time a sample takes up.
  // For instances where >1 root nodes are present, their children will show
  // their frametime percentage relative to their root's duration.
  int64_t root_duration_ns_ = 0;
  // Draws a node in the tree as a row in the table.
  // Recursively calls itself for child nodes.
  void DrawTreeNode(ProfilerSampleNode* node, int depth, int& row_index);
  const char* current_thread_ = Profiler::kMainThreadName.data();
  std::thread::id current_thread_id_;
  bool thread_set_ = false;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_CPU_HIERARCHY_PANEL_H_
