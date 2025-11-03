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

#include <array>
#include <cstdint>
#include <vector>

#include "core/performance/profiler.h"

namespace imp::editor {

struct ProfilerSampleNode {
  ProfileResult* result;
  std::vector<ProfilerSampleNode*> children;
  // uint32 since 32 bits allows for up to 4s of frame time.
  uint32_t total_time;
  int calls;
};

// Panel containing a tree view of performance profiling data.
// Each node in the tree represents a sample taken across a scope.
class HierarchyPanel {
 public:
  HierarchyPanel();
  ~HierarchyPanel();

  void DrawPanel(int frame_index);

 private:
  constexpr static int kMaxTreeDepth = 30;
  // Creates a tree of samples for the given frame index.
  // Returns nullptr if the frame index is not valid or contains no samples.
  ProfilerSampleNode* CreateTree(int frame_index);
  // Draws the tree as a table.
  void DrawTable(ProfilerSampleNode* root);
  // Draws a node in the tree as a row in the table.
  // Recursively calls itself for child nodes.
  void DrawTreeNode(ProfilerSampleNode* node, int depth, int& row_index);
  uint32_t frame_duration_;
  // Pool of nodes to be reused for each frame.
  // As long as this uses kMaxSamples from the Profiler, we will never run out.
  std::array<ProfilerSampleNode, Profiler::kMaxSamples> sample_nodes_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_CPU_HIERARCHY_PANEL_H_
