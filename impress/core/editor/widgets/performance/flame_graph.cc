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

#include "core/editor/widgets/performance/flame_graph.h"

#include <algorithm>
#include <cmath>  // For floor, ceil, log10, pow
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <thread>  // NOLINT: Needed for std::thread::id.
#include <vector>

#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/flame_graph_colors.h"
#include "core/editor/widgets/performance/frame_time_panel.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/performance/profiler.h"
#include "core/performance/profiler_structs.h"

namespace imp::editor {

namespace {
// Minimum height for the panel no matter how small the window is.
constexpr float kMinPanelHeight = 250.0f;

// The gap between rectangles in the flame graph in pixels.
constexpr float kRectGap = 1.0f;

// The height of the timeline.
constexpr float kTimelineHeight = 20.0f;

// Padding between the timeline and the graph.
constexpr float kTimelinePadding = 5.0f;

// The desired number of pixels per timeline tick.
constexpr float kDesiredPixelsPerTimelineTick = 160.0f;

// The factor by which to zoom in/out when using the mouse wheel.
constexpr float kZoomFactor = 0.01f;

// The minimum zoom level for the flame graph.
constexpr float kMinZoomLevel = 1.0f;

// The number of frames to show before and after the selected frame.
constexpr int kFrameWindow = 2;

// Distance of the tick labels to the right of their tick mark line.
constexpr float kTickLabelRightOffset = 6.0f;

// Distance of the tick labels from the timeline.
constexpr float kTickLabelTopOffset = 2.0f;

// Height of a tick mark line above the timeline.
constexpr float kTickMarkHeight = 10.0f;

// Scale factor for the sample name displayed in each flame graph node.
constexpr float kFontSizeScale = 0.8f;

// The maximum amount of pixels the text of a node can be beyond the width of
// its rectangle and still display w/ clipping. Beyond this amount, the text
// will not be displayed at all.
constexpr float kNodeLabelMaxOversize = 20.0f;

// The minimum width of a node in the flame graph to override the clipping
// threshold for text.
constexpr float kNodeOverrideTextClippingWidth = 100.0f;

// The height of the thread window when collapsed.
constexpr float kThreadWindowHeightCollapsed = 100.0f;

// Distance of the expand/collapse button from the bottom of the window.
constexpr float kButtonBottomPadding = 2.0f;

// The color of the top horizontal line in the timeline.
constexpr ImU32 kTimelineColor = IM_COL32(255, 255, 255, 150);

// The color of the tick marks in the timeline.
constexpr ImU32 kTickColor = IM_COL32(255, 255, 255, 70);

// The color of the border around a selected sample node.
constexpr ImU32 kSelectedSampleBorderColor = IM_COL32(255, 255, 255, 255);

// The color of the separator line between each flame graph window.
constexpr ImU32 kSeparatorLineColor = IM_COL32(20, 20, 20, 255);

// Background color of the worker thread label overlays.
constexpr ImU32 kWorkerThreadLabelColor = IM_COL32(0, 0, 0, 150);

// Nanoseconds in a millisecond.
constexpr float kNanosPerMs = 1000000.0f;

}  // namespace

void FlameGraph::DrawPanel(const float width, const int start_frame,
                           const int end_frame,
                           SampleProcessor& sample_processor,
                           FrameTimePanel& frame_time_panel) {
  IMP_TRACE();
  // Minimum height for the panel no matter how small the window is.
  // Flame graph takes up the remaining space in the window.
  // If the window is too small it won't be displayed so we set a min height.
  const ImVec2 available_size = ImGui::GetContentRegionAvail();
  const float child_height = std::max(kMinPanelHeight, available_size.y);

  if (ImGui::BeginChild("flame_graph_child", ImVec2(width, child_height),
                        ImGuiChildFlags_Borders,
                        ImGuiWindowFlags_NoScrollWithMouse)) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    const float row_height = ImGui::GetTextLineHeightWithSpacing();
    const ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    const ImVec2 flame_canvas_pos =
        ImVec2(canvas_pos.x, canvas_pos.y + kTimelineHeight);
    const ImVec2 flame_canvas_size =
        ImVec2(canvas_size.x, canvas_size.y - kTimelineHeight);

    // Mouse input for zoom and pan
    HandleInput(flame_canvas_pos);

    // Check to see if we've selected a single frame or a range of frames.
    const bool range_selected = start_frame != end_frame && start_frame != -1;

    // If we have a range selected, draw graphs for all the selected frames.
    // If we have a single frame selected, draw graphs for the current frame and
    // kFrameWindow frames before and after to add a bit of context.
    const int min_frame =
        range_selected ? start_frame : std::max(0, end_frame - kFrameWindow);
    const int max_frame = range_selected
                              ? end_frame
                              : std::min(Profiler::GetCurrentFrameIndex() - 1,
                                         end_frame + kFrameWindow);

    // For the timeline, the 0ms point is different depending on if we have a
    // single frame selected or a range of frames.
    // In the single frame case, the 0ms point is the start of the selected
    // frame. In the range case, 0ms is the start of the first frame in the
    // range.
    absl::StatusOr<FrameMetaData> frame_meta_data =
        Profiler::GetFrameMetaData(range_selected ? min_frame : end_frame);

    if (!frame_meta_data.ok()) {
      ImGui::EndChild();
      return;
    }

    const uint64_t selected_frame_start_time_ns =
        frame_meta_data->frame_start_time_ns;
    uint64_t total_visible_duration = 0;

    for (int i = min_frame; i <= max_frame; ++i) {
      absl::StatusOr<uint32_t> frame_duration =
          Profiler::GetTotalFrameDurationNanos(i);
      if (frame_duration.ok()) {
        total_visible_duration += *frame_duration;
      }
    }

    if (total_visible_duration == 0) {
      ImGui::EndChild();
      return;
    }

    const float time_scale = (flame_canvas_size.x * flame_graph_zoom_) /
                             static_cast<float>(total_visible_duration);

    // Clamp panning so that you can't pan past +/- 2 frames of the selected
    // one.
    const float content_screen_width =
        static_cast<float>(total_visible_duration) * time_scale;

    absl::StatusOr<FrameMetaData> min_frame_meta_data =
        Profiler::GetFrameMetaData(min_frame);

    if (!min_frame_meta_data.ok()) {
      ImGui::EndChild();
      return;
    }

    const uint64_t min_frame_start_time_ns =
        min_frame_meta_data->frame_start_time_ns;
    const uint64_t time_before_selected =
        selected_frame_start_time_ns - min_frame_start_time_ns;

    float pan_offset = static_cast<float>(time_before_selected) * time_scale;

    if (content_screen_width > flame_canvas_size.x) {
      flame_graph_pan_x_ = std::clamp(
          flame_graph_pan_x_,
          flame_canvas_size.x - content_screen_width + pan_offset, pan_offset);
    } else {
      flame_graph_pan_x_ = pan_offset;
    }

    // Shows the timeline/tick marks and their labels.
    DrawTimeline(draw_list, canvas_pos, canvas_size, time_scale);

    // Draw the nodes for each frame.
    DrawNodeArgs draw_node_args = {
        .frame_time_panel = frame_time_panel,
        .draw_list = draw_list,
        .node = nullptr,
        .depth = 0,
        .canvas_pos = flame_canvas_pos,
        .canvas_size = flame_canvas_size,
        .row_height = row_height,
        .time_scale = time_scale,
        .display_style = DisplayStyle::kNormal,
        .frame_start_time_ns = min_frame_start_time_ns,
        .selected_frame_start_time_ns = selected_frame_start_time_ns,
    };

    DrawMainThreadGraph(draw_node_args, sample_processor, min_frame, max_frame,
                        range_selected, end_frame);

    DrawWorkerThreadGraphs(sample_processor, min_frame_start_time_ns,
                           min_frame_start_time_ns + total_visible_duration,
                           draw_node_args);

    // If a single frame is selected draw boundary lines around it.
    if (!range_selected) {
      absl::StatusOr<uint32_t> frame_duration =
          Profiler::GetTotalFrameDurationNanos(end_frame);

      if (frame_duration.ok()) {
        DrawFrameBoundaryLines(draw_list, canvas_pos, canvas_size, time_scale,
                               selected_frame_start_time_ns, *frame_duration);
      }
    }
  }
  ImGui::EndChild();
}

void FlameGraph::DrawMainThreadGraph(DrawNodeArgs& args,
                                     SampleProcessor& sample_processor,
                                     int min_frame, int max_frame,
                                     bool range_selected,
                                     int selected_frame_index) {
  // Calculate the height of the thread window by figuring out the max depth
  // of all frames currently displayed in the window.
  size_t max_depth = 0;
  for (int i = min_frame; i <= max_frame; ++i) {
    max_depth =
        std::max(max_depth, sample_processor.GetProcessedFrame(i).max_depth);
  }

  const std::thread::id main_thread_id = Profiler::GetMainThreadId();

  // Set the main thread to be expanded by default here.
  auto it = thread_expanded_state_.find(main_thread_id);
  if (it == thread_expanded_state_.end()) {
    it = thread_expanded_state_.insert({main_thread_id, true}).first;
  }

  bool& expanded = it->second;

  // Calculate the display height of the thread window.
  // If the graph's depth causes it to exceed the collapsed height, show a
  // button to expand it. If the graph's depth is less than the collapsed
  // height, give it as much space as it needs.
  // Add 2 to the max depth to account for the extra frametime row in the main
  // thread graph. Worker threads add 1.
  const float full_thread_height = (max_depth + 2) * args.row_height;
  const bool show_button = full_thread_height > kThreadWindowHeightCollapsed;
  float thread_window_height = full_thread_height;

  if (show_button && !expanded)
    thread_window_height = kThreadWindowHeightCollapsed;

  // Create a new window for the main thread graph and draw frames into it.
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  if (ImGui::BeginChild("main_thread_child", ImVec2(0, thread_window_height),
                        ImGuiChildFlags_None,
                        ImGuiWindowFlags_NoScrollWithMouse)) {
    // If we have a single frame selected, fade out the frames before and after
    // the selected frame to make it easier to see the selected frame.
    // If we have a range selected, don't fade anything.
    for (int i = min_frame; i <= max_frame; ++i) {
      const DisplayStyle display_style =
          !range_selected && (i != selected_frame_index)
              ? DisplayStyle::kFaded
              : DisplayStyle::kNormal;
      DrawFrame(args, sample_processor, i, display_style);
    }

    if (show_button) {
      DrawExpandCollapseButton(expanded);
    }
  }
  ImGui::EndChild();
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding

  DrawSeparator(args.draw_list);
}

void FlameGraph::DrawFrame(DrawNodeArgs& args,
                           SampleProcessor& sample_processor, int frame_index,
                           DisplayStyle display_style) {
  args.draw_list = ImGui::GetWindowDrawList();
  const ProcessedSamples& frame_to_draw =
      sample_processor.GetProcessedFrame(frame_index);
  absl::StatusOr<uint32_t> frame_duration =
      Profiler::GetTotalFrameDurationNanos(frame_index);

  if (!frame_duration.ok()) return;

  // Create a fake root node for the entire frametime w/ vysnc
  args.display_style = display_style;

  DrawNodeArgs rect_args = args;
  rect_args.depth = 0;
  rect_args.display_style = display_style;
  Rect rect;
  const bool is_frame_visible =
      DrawRectangle(rect_args, "Frametime w/ Vsync", args.frame_start_time_ns,
                    args.frame_start_time_ns + *frame_duration, rect);

  // If the overall frame rectangle is not visible, none of the samples that
  // correspond to it will be either so we can avoid doing any more work.
  // Recursively draw the samples for this frame from the specified thread.
  if (is_frame_visible) {
    for (const auto* root : frame_to_draw.sample_roots) {
      args.node = root;
      args.depth = 1;
      DrawFlameGraphNode(args);
    }
  }
  args.frame_start_time_ns += *frame_duration;
}

void FlameGraph::DrawTooltip(const absl::string_view name,
                             const uint64_t total_time_ns,
                             const size_t total_memory_allocated,
                             const size_t total_memory_allocations_count) {
  ImGui::BeginTooltip();
  const float duration_ms = static_cast<float>(total_time_ns / kNanosPerMs);
  const size_t allocated = total_memory_allocated;
  constexpr size_t kKilobyte = 1024;
  constexpr size_t kMegabyte = 1024 * 1024;
  char label_str[32];

  if (allocated >= kMegabyte) {
    absl::SNPrintF(label_str, sizeof(label_str), "%.1f MB",
                   static_cast<float>(allocated) / kMegabyte);
  } else if (allocated >= kKilobyte) {
    absl::SNPrintF(label_str, sizeof(label_str), "%.1f KB",
                   static_cast<float>(allocated) / kKilobyte);
  } else {
    absl::SNPrintF(label_str, sizeof(label_str), "%zu B", allocated);
  }
  ImGui::Text("%s\n%.3f ms\n%zu Allocations\n%s Allocated", name.data(),
              duration_ms, total_memory_allocations_count, label_str);
  ImGui::EndTooltip();
}

void FlameGraph::DrawFlameGraphNode(DrawNodeArgs& args) {
  const SampleNode* node = args.node;
  if (!node) return;

  const int depth = args.depth;

  const uint64_t absolute_start_time_ns =
      args.frame_start_time_ns +
      static_cast<uint64_t>(node->result->GetStartTimeNanos());
  const uint64_t absolute_end_time_ns =
      args.frame_start_time_ns +
      static_cast<uint64_t>(node->result->GetEndTimeNanos());

  const absl::string_view name = node->result->GetName();

  // Draw the rectangle for this node.
  // If the rectangle gets culled this returns false and we return early.
  Rect rect;
  if (!DrawRectangle(args, name.data(), absolute_start_time_ns,
                     absolute_end_time_ns, rect)) {
    return;
  }

  // Select this sample if the rectangle is clicked.
  if (ImGui::IsMouseHoveringRect(rect.min, rect.max) &&
      ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    args.frame_time_panel.SetSelectedSampleName(name);
  }

  // Hover Tooltip
  if (ImGui::IsMouseHoveringRect(rect.min, rect.max)) {
    DrawTooltip(name, node->total_time_ns, node->total_memory_allocated,
                node->total_memory_allocations_count);
  }

  // If this is the selected sample, draw a white border around it.
  if (name == args.frame_time_panel.GetSelectedSampleName()) {
    args.draw_list->AddRect(rect.min, rect.max, kSelectedSampleBorderColor,
                            0.0f, ImDrawFlags_None, 2.0f);
  }

  // Recursively draw children.
  const SampleNode* child = node->first_child;
  while (child != nullptr) {
    args.node = child;
    args.depth = depth + 1;
    DrawFlameGraphNode(args);
    child = child->next_sibling;
  }
}

void FlameGraph::DrawWorkerThreadGraphs(SampleProcessor& sample_processor,
                                        uint64_t start_time_ns,
                                        uint64_t end_time_ns,
                                        DrawNodeArgs& draw_node_args) {
  // Gets every profile result for every worker thread for the duration.
  // This includes samples that start/finish outside of the range but overlap
  // during a portion of it.
  const RawWorkerSamplesMap raw_worker_samples_map =
      Profiler::GetAllWorkerThreadsSamples(start_time_ns, end_time_ns);

  const ProcessedWorkerSamplesMap worker_samples_map =
      sample_processor.ProcessAllWorkerThreadsSamples(raw_worker_samples_map);

  const std::vector<std::thread::id> worker_thread_ids =
      Profiler::GetWorkerThreadIds();

  const size_t worker_thread_count = worker_thread_ids.size();
  std::thread::id thread_id;
  const std::thread::id main_thread_id = Profiler::GetMainThreadId();

  for (size_t i = 0; i < worker_thread_count; ++i) {
    thread_id = worker_thread_ids[i];

    if (thread_id == main_thread_id) continue;
    const auto it = worker_samples_map.find(thread_id);
    if (it == worker_samples_map.end()) continue;

    DrawWorkerThreadGraph(thread_id, draw_node_args, it->second);
  }
}

void FlameGraph::DrawWorkerThreadGraph(const std::thread::id thread_id,
                                       DrawNodeArgs& args,
                                       const ProcessedSamples& samples) {
  char thread_name[64];
  absl::SNPrintF(thread_name, sizeof(thread_name), "Worker Thread %d",
                 absl::Hash<std::thread::id>{}(thread_id));

  // Calculate the display height of the thread window.
  // If the graph exceeds the collapsed height, show a button to expand it.
  // If the graph height is less than the collapsed height, give it as much
  // space as it needs.
  bool& expanded = thread_expanded_state_[thread_id];
  const float full_thread_height = (samples.max_depth + 1) * args.row_height;
  const bool show_button = full_thread_height > kThreadWindowHeightCollapsed;

  float thread_window_height = full_thread_height;

  if (show_button && !expanded)
    thread_window_height = kThreadWindowHeightCollapsed;

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::BeginChild(thread_name, ImVec2(0, thread_window_height),
                    ImGuiChildFlags_None, ImGuiWindowFlags_NoScrollWithMouse);
  args.draw_list = ImGui::GetWindowDrawList();
  args.canvas_pos = ImGui::GetCursorScreenPos();
  args.canvas_size = ImGui::GetContentRegionAvail();

  const size_t root_count = samples.sample_roots.size();
  for (size_t i = 0; i < root_count; i++) {
    args.node = samples.sample_roots[i];
    args.depth = 0;
    DrawWorkerGraphNode(args);
  }

  // Display worker thread name at the top-left of the window.
  // This is done after the graph is drawn so that it ends up on top.
  DrawWorkerThreadLabel(args.draw_list, thread_name,
                        ImGui::GetCursorScreenPos());

  if (show_button) {
    DrawExpandCollapseButton(expanded);
  }

  ImGui::EndChild();
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding

  DrawSeparator(args.draw_list);
}

void FlameGraph::DrawWorkerGraphNode(DrawNodeArgs& args) {
  const SampleNode* node = args.node;
  if (!node) return;

  const int depth = args.depth;

  const absl::string_view name = node->result->GetName();
  const uint64_t start_time = node->result->GetStartTimeNanos();
  const uint64_t end_time = node->result->GetEndTimeNanos();

  DrawNodeArgs rect_args = args;
  rect_args.display_style = DisplayStyle::kNormal;
  Rect rect;
  if (!DrawRectangle(rect_args, name.data(), start_time, end_time, rect)) {
    return;
  }

  // Hover Tooltip
  if (ImGui::IsMouseHoveringRect(rect.min, rect.max)) {
    DrawTooltip(name, node->total_time_ns, node->total_memory_allocated,
                node->total_memory_allocations_count);
  }

  // Cannot select samples in worker threads so that is not present here.

  // Recursively draw children.
  const SampleNode* child = node->first_child;
  while (child != nullptr) {
    args.node = child;
    args.depth = depth + 1;
    DrawWorkerGraphNode(args);
    child = child->next_sibling;
  }
}

void FlameGraph::DrawTimeline(ImDrawList* draw_list, const ImVec2 canvas_pos,
                              const ImVec2 canvas_size,
                              const float time_scale) {
  const ImVec2 timeline_pos = canvas_pos;
  const float timeline_line_y =
      timeline_pos.y + kTimelineHeight - kTimelinePadding;

  // Horizontal line for the timeline.
  draw_list->AddLine(ImVec2(timeline_pos.x, timeline_line_y),
                     ImVec2(timeline_pos.x + canvas_size.x, timeline_line_y),
                     kTimelineColor);

  // Calculate the time interval between ticks for the flame graph.
  // If we were at a zoom level where we can see 20ms of total time and our tick
  // interval was 0.1ms there would be 200 ticks and time labels.
  // This would be far too many but if we were zoomed into 1ms of total
  // frametime that interval would be perfect.
  // Here we dynamically set the interval based on our current zoom level to
  // keep the number of ticks reasonable.
  const float time_interval_ns = kDesiredPixelsPerTimelineTick / time_scale;
  const double magnitude = pow(10.0, floor(log10(time_interval_ns)));
  const double residual = time_interval_ns / magnitude;
  double nice_interval_ns;
  if (residual < 1.5)
    nice_interval_ns = 1.0 * magnitude;
  else if (residual < 3.5)
    nice_interval_ns = 2.0 * magnitude;
  else if (residual < 7.5)
    nice_interval_ns = 5.0 * magnitude;
  else
    nice_interval_ns = 10.0 * magnitude;

  // Prevent too small of an interval.
  nice_interval_ns =
      std::max(nice_interval_ns, 1000.0);  // At least 1 microsecond

  // Calculate the time range visible on screen, relative to the selected frame
  // start time
  const int64_t view_start_relative_ns =
      static_cast<int64_t>(-flame_graph_pan_x_ / time_scale);
  const int64_t view_end_relative_ns =
      static_cast<int64_t>((canvas_size.x - flame_graph_pan_x_) / time_scale);

  // Find the first tick mark to draw
  const int64_t first_tick_relative_ns =
      floor(static_cast<double>(view_start_relative_ns) / nice_interval_ns) *
      static_cast<int64_t>(nice_interval_ns);

  // Draw the tick marks and labels.
  for (int64_t tick_time_relative_ns = first_tick_relative_ns;
       tick_time_relative_ns <= view_end_relative_ns;
       tick_time_relative_ns += static_cast<int64_t>(nice_interval_ns)) {
    const float x = canvas_pos.x + flame_graph_pan_x_ +
                    static_cast<float>(tick_time_relative_ns) * time_scale;

    if (x < canvas_pos.x || x > canvas_pos.x + canvas_size.x) continue;

    draw_list->AddLine(ImVec2(x, timeline_line_y - kTickMarkHeight),
                       ImVec2(x, timeline_pos.y + canvas_size.y), kTickColor);

    char tick_label[32];
    const float tick_ms =
        static_cast<float>(tick_time_relative_ns) / kNanosPerMs;
    absl::SNPrintF(tick_label, sizeof(tick_label), "%.1fms", tick_ms);

    const ImVec2 text_size = ImGui::CalcTextSize(tick_label);

    draw_list->AddText(
        ImVec2(x + kTickLabelRightOffset,
               timeline_line_y - text_size.y - kTickLabelTopOffset),
        kTimelineColor, tick_label);
  }
}

void FlameGraph::HandleInput(const ImVec2& flame_canvas_pos) {
  if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows)) return;
  ImGuiIO& io = ImGui::GetIO();

  // Zoom
  float wheel = io.MouseWheel;
  if (wheel != 0) {
    float zoom_factor = 1.0f + wheel * kZoomFactor;
    float old_zoom = flame_graph_zoom_;
    flame_graph_zoom_ *= zoom_factor;
    flame_graph_zoom_ = std::max(kMinZoomLevel, flame_graph_zoom_);

    // Adjust pan value so that zooming is centered on the mouse cursor.
    float mouse_x_relative = io.MousePos.x - flame_canvas_pos.x;
    float pan_delta = (mouse_x_relative - flame_graph_pan_x_) *
                      (flame_graph_zoom_ / old_zoom - 1.0f);
    flame_graph_pan_x_ -= pan_delta;
  }

  // Pan
  if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
    flame_graph_pan_x_ += io.MouseDelta.x;
  }
}

bool FlameGraph::DrawRectangle(const DrawNodeArgs& args, const char* name,
                               const uint64_t start_time,
                               const uint64_t end_time, Rect& rect) {
  // Calculate the x/y position and width of the node in the flame graph.
  const int64_t time_offset =
      static_cast<int64_t>(start_time) -
      static_cast<int64_t>(args.selected_frame_start_time_ns);
  const float width =
      static_cast<float>(end_time - start_time) * args.time_scale;
  const float x = args.canvas_pos.x + flame_graph_pan_x_ +
                  static_cast<float>(time_offset) * args.time_scale;
  const float y = args.canvas_pos.y + args.depth * args.row_height;

  // Return early if the rectangle would be outside the canvas.
  if (y + args.row_height < args.canvas_pos.y ||
      y > args.canvas_pos.y + args.canvas_size.y ||
      x + width < args.canvas_pos.x ||
      x > args.canvas_pos.x + args.canvas_size.x) {
    return false;
  }

  // Vectors for the diagonal corners representing the rectangle.
  rect.min.x = x;
  rect.min.y = y;
  rect.max.x = x + width;
  rect.max.y = y + args.row_height;

  // Clamp the rectangle bounds to the canvas bounds.
  rect.min.x = std::max(rect.min.x, args.canvas_pos.x);
  rect.min.y = std::max(rect.min.y, args.canvas_pos.y);
  rect.max.x = std::min(rect.max.x, args.canvas_pos.x + args.canvas_size.x);
  rect.max.y = std::min(rect.max.y, args.canvas_pos.y + args.canvas_size.y);

  // Add a small gap between rectangles.
  rect.max.x -= kRectGap;
  rect.max.y -= kRectGap;

  // The visible area of the rectangle is too small to display.
  // This is influenced by a few factors:
  // 1. Zoom Level - When zooming out sample rectangles naturally get smaller.
  // 2. Sample Duration - Short samples create smaller rectangles.
  // 3. Clamping - If the sample is near the edge of the visible area, it
  // might not be big enough to display after subtracting the gap.
  // By returning here we avoid drawing imperceptible samples.
  if (rect.max.x <= rect.min.x || rect.max.y <= rect.min.y) return false;

  // Get a random but name-stable color for the sample.
  ImU32 color = GetColorForName(name, args.display_style);
  args.draw_list->AddRectFilled(rect.min, rect.max, color);

  // Draw text if the rectangle is wide enough to display it.
  const float font_size = ImGui::GetFontSize() * kFontSizeScale;
  ImVec2 text_size = ImGui::CalcTextSize(name, nullptr, false, 0.0f);
  text_size.x *= kFontSizeScale;
  text_size.y *= kFontSizeScale;

  // If the text is larger than the rectangle, but not by more than the
  // threshold OR the rectangle is sufficiently wide, draw the text.
  // This makes it so that the text is displayed fully if there's enough room,
  // but otherwise displays it clipped to the rectangle.
  // If the rectangle is too small, the text is not displayed at all.
  size_t rect_width = rect.max.x - rect.min.x;

  if (text_size.x < rect_width + kNodeLabelMaxOversize ||
      rect_width > kNodeOverrideTextClippingWidth) {
    ImVec2 text_pos(
        rect.min.x + ((rect.max.x - rect.min.x) - text_size.x) * 0.5f,
        rect.min.y + (args.row_height - text_size.y) * 0.5f);
    // Text clipping
    ImVec4 clip_rect(rect.min.x, rect.min.y, rect.max.x, rect.max.y);
    args.draw_list->AddText(ImGui::GetFont(), font_size, text_pos,
                            IM_COL32_WHITE, name, nullptr, 0.0f, &clip_rect);
  }

  return true;  // Rectangle is visible.
}

void FlameGraph::DrawExpandCollapseButton(bool& expanded) {
  const char* button_text = expanded ? "Collapse" : "Expand";
  ImVec2 button_size = ImGui::CalcTextSize(button_text);
  button_size.x += ImGui::GetStyle().FramePadding.x * 2;
  button_size.y += ImGui::GetStyle().FramePadding.y * 2;

  const ImVec2 window_size = ImGui::GetWindowSize();
  const ImVec2 button_pos = ImVec2(
      ImGui::GetCursorScreenPos().x + (window_size.x - button_size.x) * 0.5f,
      ImGui::GetCursorScreenPos().y + window_size.y - button_size.y -
          kButtonBottomPadding);
  ImGui::SetCursorScreenPos(button_pos);

  if (ImGui::Button(button_text)) {
    expanded = !expanded;
  }
}

void FlameGraph::DrawSeparator(ImDrawList* draw_list) {
  const ImVec2 p1 = ImGui::GetItemRectMin();
  const ImVec2 p2 = ImGui::GetItemRectMax();
  draw_list->AddLine(ImVec2(p1.x, p2.y), ImVec2(p2.x, p2.y),
                     kSeparatorLineColor, 2.0f);
}

void FlameGraph::DrawWorkerThreadLabel(ImDrawList* draw_list,
                                       const char* thread_name,
                                       ImVec2 child_pos) {
  const ImVec2 text_size = ImGui::CalcTextSize(thread_name);
  const ImVec2 text_padding = ImVec2(4, 2);
  const ImVec2 rect_size = ImVec2(text_size.x + text_padding.x * 2,
                                  text_size.y + text_padding.y * 2);
  const ImVec2 rect_min = child_pos;
  const ImVec2 rect_max =
      ImVec2(child_pos.x + rect_size.x, child_pos.y + rect_size.y);

  draw_list->AddRectFilled(rect_min, rect_max, kWorkerThreadLabelColor);
  draw_list->AddText(
      ImVec2(child_pos.x + text_padding.x, child_pos.y + text_padding.y),
      IM_COL32_WHITE, thread_name);
}

void FlameGraph::DrawFrameBoundaryLines(ImDrawList* draw_list,
                                        ImVec2 canvas_pos, ImVec2 canvas_size,
                                        float time_scale,
                                        uint64_t selected_frame_start_time_ns,
                                        uint64_t selected_frame_duration_ns) {
  // Calculate the x position for the start and end lines.
  float start_x = canvas_pos.x + flame_graph_pan_x_;
  float end_x = canvas_pos.x + flame_graph_pan_x_ +
                static_cast<float>(selected_frame_duration_ns) * time_scale;

  // Clamp lines to the canvas bounds.
  start_x = std::max(start_x, canvas_pos.x);
  end_x = std::min(end_x, canvas_pos.x + canvas_size.x);

  // Draw the start line.
  if (start_x >= canvas_pos.x && start_x <= canvas_pos.x + canvas_size.x) {
    draw_list->AddLine(ImVec2(start_x, canvas_pos.y),
                       ImVec2(start_x, canvas_pos.y + canvas_size.y),
                       IM_COL32_WHITE);
  }

  // Draw the end line.
  if (end_x >= canvas_pos.x && end_x <= canvas_pos.x + canvas_size.x) {
    draw_list->AddLine(ImVec2(end_x, canvas_pos.y),
                       ImVec2(end_x, canvas_pos.y + canvas_size.y),
                       IM_COL32_WHITE);
  }
}

ImU32 FlameGraph::GetColorForName(const absl::string_view name,
                                  const DisplayStyle display_style) {
  // Hash the name of the sample and then mod it by the number of colors to get
  // a consistent color across all samples with the same name.
  const size_t hash = absl::Hash<absl::string_view>{}(name);
  const size_t index = hash % flame_graph_colors::kNumColors;
  if (display_style == DisplayStyle::kFaded) {
    return flame_graph_colors::kUnselectedFrameColors[index];
  }
  return flame_graph_colors::kFrameColors[index];
}
}  // namespace imp::editor
