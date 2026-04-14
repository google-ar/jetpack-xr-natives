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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_HIERARCHY_PANEL_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_HIERARCHY_PANEL_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>  // NOLINT: Need to use std::thread::id.
#include <vector>

#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/performance/profiler.h"
#include "core/performance/profiler_structs.h"

namespace imp::editor {

class FrameTimePanel;

// Panel containing a tree view of performance profiling data.
// Each node in the tree represents a sample taken across a scope.
// Rendered as a subpanel of the FrameTimePanel.
class HierarchyPanel {
 public:
  HierarchyPanel() {}
  ~HierarchyPanel() = default;

  void DrawPanel(float width, int start_frame, int end_frame,
                 SampleProcessor& sample_processor,
                 FrameTimePanel& frame_time_panel);

  std::thread::id GetSelectedThreadId() const { return current_thread_id_; }

 private:
  constexpr static int kMaxTreeDepth = 30;
  // The total duration of the root node for the current tree.
  // This is used to calculate the percentage of time a sample takes up.
  // For instances where >1 root nodes are present, their children will show
  // their frametime percentage relative to their root's duration.
  int64_t root_duration_ns_ = 0;
  // Draws a node in the tree as a row in the table.
  // Recursively calls itself for child nodes.
  void DrawTreeNode(FrameTimePanel& frame_time_panel, SampleNode* node,
                    int depth, int& row_index);
  void DrawWorkerTreeNode(FrameTimePanel& frame_time_panel, SampleNode* node,
                          int depth, int& row_index);
  bool DrawMainThreadSamples(int start_frame, int end_frame,
                             SampleProcessor& sample_processor,
                             FrameTimePanel& frame_time_panel);
  bool DrawMainThreadFrame(int frame_index, SampleProcessor& sample_processor,
                           FrameTimePanel& frame_time_panel);
  bool DrawWorkerThreadSamples(int start_frame, int end_frame,
                               SampleProcessor& sample_processor,
                               std::thread::id thread_id,
                               FrameTimePanel& frame_time_panel);
  void DrawTableRow(FrameTimePanel& frame_time_panel, const char* name,
                    uint32_t time, int calls, size_t memory_allocated,
                    size_t allocations_count, int& row_index);
  void DrawThreadSelector();
  // Returns a hard copy of a tree of samples.
  // This is used to modify the tree without affecting the original data.
  // Original data is owned by the SampleProcessor and will not be modified
  // by this class.
  std::vector<SampleNode*> GetTreeHardCopy(
      const std::vector<SampleNode*>& roots, NodePool& node_pool);
  // Copies a node to the pool of sample nodes
  // Recursively copies all child nodes.
  SampleNode* CopyNodeToPool(const SampleNode* node, NodePool& node_pool);
  std::unique_ptr<SampleNode> CreateFakeRootSample(
      const std::vector<SampleNode*>& roots, uint64_t start_time,
      uint64_t end_time, WorkerProfileResult& fake_result);

  // Pool of nodes to be reused each time a tree is drawn.
  MainThreadNodePool main_thread_node_pool_;

  // For thread selection:
  const char* current_thread_ = Profiler::kMainThreadName.data();
  std::thread::id current_thread_id_;
  bool thread_set_ = false;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_HIERARCHY_PANEL_H_
