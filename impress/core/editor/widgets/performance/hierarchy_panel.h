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
#include <memory>
#include <vector>

#include "core/performance/profiler.h"

namespace imp::editor {

struct ProfilerSampleNode {
  ProfileResult* result;
  std::vector<std::unique_ptr<ProfilerSampleNode>> children;
  int64_t total_time;
  int calls;
};

// A panel for the performance monitor to show the various information on the
// lifetime of a frame
class HierarchyPanel {
 public:
  HierarchyPanel();
  ~HierarchyPanel();

  void DrawPanel(int frame_index);

 private:
  constexpr static int kMaxTreeDepth = 30;
  void DrawTree(ProfilerSampleNode* node, int depth, int& row_index);
  int64_t frame_duration_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_CPU_HIERARCHY_PANEL_H_
