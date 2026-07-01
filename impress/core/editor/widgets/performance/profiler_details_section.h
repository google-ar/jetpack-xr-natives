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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_PROFILER_DETAILS_SECTION_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_PROFILER_DETAILS_SECTION_H_

#include <string>

#include "core/editor/widgets/performance/callstack_panel.h"
#include "core/editor/widgets/performance/flame_graph.h"
#include "core/editor/widgets/performance/hierarchy_panel.h"

namespace imp::editor {

class ProfilerDataProvider;

// Handles the detailed profiling information section of the Performance Window,
// including the Hierarchy and Flame Graph views.
class ProfilerDetailsSection {
 public:
  explicit ProfilerDetailsSection(ProfilerDataProvider& data_provider);
  ~ProfilerDetailsSection() = default;

  void Draw();

 private:
  enum class ViewMode { kHierarchy, kFlameGraph };

  void DrawOptionsBar();
  void DrawSplitter();
  void DrawThreadSelector();

  ProfilerDataProvider& data_provider_;
  HierarchyPanel hierarchy_panel_;
  CallstackPanel callstack_panel_;
  FlameGraph flame_graph_;

  ViewMode view_mode_ = ViewMode::kHierarchy;
  int show_callstack_ = 0;
  float sample_view_width_ = -1.0f;
  float callstack_panel_width_ = 450.0f;
  bool thread_set_ = false;
  std::string search_query_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_PROFILER_DETAILS_SECTION_H_
