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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_FLAME_GRAPH_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_FLAME_GRAPH_H_

#include <cstdint>
#include <thread>  // NOLINT: We show graphs by thread id.

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"

namespace imp::editor {

class FrameTimePanel;

class FlameGraph {
 public:
  FlameGraph() = default;
  ~FlameGraph() = default;

  void DrawPanel(int frame_index, SampleProcessor& sample_processor,
                 FrameTimePanel& frame_time_panel);

 private:
  struct Rect {
    ImVec2 min;
    ImVec2 max;
  };

  // Arguments for DrawFlameGraphNode.
  // Constantly updated and passed by reference to reduce stack pressure.
  struct DrawNodeArgs {
    FrameTimePanel& frame_time_panel;
    ImDrawList* draw_list;
    const SampleNode* node;
    int depth;
    ImVec2 canvas_pos;
    ImVec2 canvas_size;
    float row_height;
    float time_scale;
    bool is_current_frame;
    uint64_t frame_start_time_ns;
    uint64_t selected_frame_start_time_ns;
  };

  // Draws a rectangle in the flame graph. Width is based on time elapsed.
  // Y-position is based on depth.
  // Returns true if the rectangle was drawn, false if it was clipped.
  bool DrawRectangle(ImDrawList* draw_list, const char* name, int depth,
                     ImVec2 canvas_pos, ImVec2 canvas_size, float time_scale,
                     float row_height, uint64_t start_time, uint64_t end_time,
                     uint64_t selected_frame_start_time_ns,
                     bool is_current_frame, Rect& rect);

  // Draws a node in the flame graph.
  // Recursively calls itself for child nodes.
  void DrawFlameGraphNode(DrawNodeArgs& args);

  // Draws a frame in the flame graph. Frames other than the selected frame are
  // drawn with reduced opacity to make it more obvious which frame is selected.
  void DrawFrame(DrawNodeArgs& args, SampleProcessor& sample_processor,
                 int frame_index, int selected_frame_index);

  // Draws the timeline for the flame graph.
  void DrawTimeline(ImDrawList* draw_list, ImVec2 canvas_pos,
                    ImVec2 canvas_size, float time_scale);

  // Handles mouse input for zooming and panning.
  void HandleInput(const ImVec2& flame_canvas_pos);

  // Draws lines at the start and end of the selected frame.
  void DrawFrameBoundaryLines(ImDrawList* draw_list, ImVec2 canvas_pos,
                              ImVec2 canvas_size, float time_scale,
                              uint64_t selected_frame_start_time_ns,
                              uint64_t selected_frame_duration_ns);

  // Draws a flame graph for the main thread.
  void DrawMainThreadGraph(DrawNodeArgs& args,
                           SampleProcessor& sample_processor, int min_frame,
                           int max_frame, int selected_frame_index);

  // Draws all worker threads graphs.
  void DrawWorkerThreadGraphs(SampleProcessor& sample_processor,
                              uint64_t start_time_ns, uint64_t end_time_ns,
                              DrawNodeArgs& draw_node_args);

  // Draws a flame graph for a specific set of samples.
  void DrawWorkerThreadGraph(std::thread::id thread_id, DrawNodeArgs& args,
                             ProcessedSamples& samples);

  // Draws a node in the flame graph for a specific set of samples.
  // Recursively calls itself for child nodes.
  void DrawWorkerGraphNode(DrawNodeArgs& args);

  // Draws an overlay label in a worker thread's flame graph showing its name.
  void DrawWorkerThreadLabel(ImDrawList* draw_list, const char* thread_name,
                             ImVec2 child_pos);

  // Draws a separator line between flame graph windows.
  void DrawSeparator(ImDrawList* draw_list);

  // Draws an expand/collapse button for a thread window.
  void DrawExpandCollapseButton(bool& expanded);

  // Helper to get a color based on the name.
  ImU32 GetColorForName(absl::string_view name, bool is_current_frame);

  // Current zoom level of the flame graph.
  float flame_graph_zoom_ = 1.0f;

  // Current pan offset of the flame graph, in pixels.
  float flame_graph_pan_x_ = 0.0f;

  // Whether the thread window is expanded or collapsed.
  absl::flat_hash_map<std::thread::id, bool> thread_expanded_state_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_FLAME_GRAPH_H_
