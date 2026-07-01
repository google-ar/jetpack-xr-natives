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

#include <cstdint>
#include <memory>
#include <string>
#include <thread>  // NOLINT: Need to use std::thread::id.
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/performance/profiler_structs.h"

namespace imp::editor {

class ProfilerDataProvider;

// Panel containing a tree view of performance profiling data.
// Each node in the tree represents a sample taken across a scope.
class HierarchyPanel {
 public:
  HierarchyPanel() {}
  ~HierarchyPanel() = default;

  void DrawPanel(float width, ProfilerDataProvider& data_provider,
                 std::thread::id thread_id, absl::string_view search_query);

 private:
  constexpr static int kMaxTreeDepth = 30;

  // Arguments for DrawTreeNode.
  // Constantly updated and passed by reference to reduce stack pressure.
  struct HierarchyDrawArgs {
    ProfilerDataProvider& data_provider;
    std::thread::id thread_id;
    int& row_index;
    // The total duration of the root node for the current tree.
    // This is used to calculate the percentage of time a sample takes up.
    int64_t root_duration_ns;
    SampleNode* node;
    int depth;
    absl::string_view search_query;
    bool should_expand_selected;
    absl::string_view selected_sample_name;
    absl::flat_hash_map<uint32_t, bool>& saved_node_states;
    bool should_restore_states;
  };

  // Draws a node in the tree as a row in the table.
  // Recursively calls itself for child nodes.
  void DrawTreeNode(HierarchyDrawArgs& args);
  bool DrawMainThreadSamples(ProfilerDataProvider& data_provider,
                             absl::string_view search_query);
  bool DrawMainThreadFrame(int frame_index, SampleProcessor& sample_processor,
                           ProfilerDataProvider& data_provider,
                           absl::string_view search_query);
  bool DrawWorkerThreadSamples(std::thread::id thread_id,
                               ProfilerDataProvider& data_provider,
                               absl::string_view search_query);
  void DrawTableRow(HierarchyDrawArgs& args);
  // Returns a hard copy of a tree of samples.
  // This is used to modify the tree without affecting the original data.
  // Original data is owned by the SampleProcessor and will not be modified
  // by this class.
  std::vector<SampleNode*>& GetTreeHardCopy(
      const std::vector<SampleNode*>& roots, NodePool& node_pool);
  // Copies a node to the pool of sample nodes
  // Recursively copies all child nodes.
  SampleNode* CopyNodeToPool(const SampleNode* node, NodePool& node_pool);
  std::unique_ptr<SampleNode> CreateFakeRootSample(
      const std::vector<SampleNode*>& roots, uint64_t start_time,
      uint64_t end_time, WorkerProfileResult& fake_result);
  bool ShouldNodeBeOpen(const HierarchyDrawArgs& args, SampleNode* node,
                        uint32_t id) const;

  // Pool of nodes to be reused each time a tree is drawn.
  MainThreadNodePool main_thread_node_pool_;
  absl::flat_hash_map<uint32_t, bool> saved_node_states_;
  std::string last_search_query_;
  bool should_restore_states_ = false;
  bool should_expand_selected_ = false;
  bool selected_during_search_ = false;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_HIERARCHY_PANEL_H_
