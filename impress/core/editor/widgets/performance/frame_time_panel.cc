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

#include "core/editor/widgets/performance/frame_time_panel.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <thread>  // NOLINT: Need to sort things by thread id.
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "dear_imgui/imgui.h"
#include "implot/implot.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/circular_buffer.h"
#include "core/editor/widgets/performance/config.h"
#include "core/editor/widgets/performance/hierarchy_panel.h"
#include "core/editor/widgets/performance/imgui_helper.h"
#include "core/editor/widgets/performance/profiler_data_provider.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/performance/profiler.h"
#include "core/performance/profiler_state.h"
#include "core/view/base_view.h"

namespace imp::editor {

namespace {

// Width of the legend to the left of the frame time plot.
constexpr float kLegendWidth = 150.0f;

// Custom Y-axis ticks for common frame rate targets
constexpr double kTicks[] = {
    1000.0 / 30.0,   // 30 FPS
    1000.0 / 60.0,   // 60 FPS
    1000.0 / 90.0,   // 90 FPS
    1000.0 / 120.0,  // 120 FPS
    1000.0 / 144.0,  // 144 FPS
    1000.0 / 240.0   // 240 FPS
};

constexpr const char* kTickLabels[] = {"33.3ms (30fps)", "16.7ms (60fps)",
                                       "11.1ms (90fps)", "8.1ms (120fps)",
                                       "6.9ms (144fps)", "4.2ms (240fps)"};

// How many tick labels to display at once.
// Running at 30fps you will see the tick labels for 60, 90, and 30fps.
// At 120 you will see the labels for 120, 144, and 240fps.
constexpr int kNumTickLabels = 3;

// Nanoseconds in a millisecond.
constexpr float kNanosPerMs = 1000000.0f;

// Lower bound for the frame time plot.
constexpr float kLowerFrameBound = 0.0f;

// Padding from the left edge of the plot to the tick label text.
constexpr float kTickLabelLeftPadding = 20.0f;

// Horizontal margin around the tick label text rectangle.
constexpr float kTickLabelRectMarginWidth = 4.0f;

// Vertical margin around the tick label text rectangle.
constexpr float kTickLabelRectMarginHeight = 2.0f;
}  // namespace

FrameTimePanel::FrameTimePanel(ProfilerDataProvider& data_provider,
                               BaseView& view, int buffer_size)
    : data_provider_(data_provider),
      view_(view),
      buffer_(buffer_size),
      view_config_(view.GetConfig()) {}

FrameTimePanel::~FrameTimePanel() = default;

float FrameTimePanel::GetHighestVisibleFrameTimeMS(int time_span_seconds) {
  int earliest_visible_frame =
      Profiler::GetCurrentFrameIndex() -
      time_span_seconds * details::kNumDisplayValuesPerSecond;
  float highest_frame_time_ms = 0.0f;
  for (const auto& frame_time_info : buffer_.data()) {
    if (frame_time_info.frame_number < earliest_visible_frame) {
      continue;
    }

    if (frame_time_info.frame_time_ms >= highest_frame_time_ms) {
      highest_frame_time_ms = frame_time_info.frame_time_ms;
    }
  }
  return highest_frame_time_ms;
}

void FrameTimePanel::DrawLegend(float width, float height) {
  if (ImGui::BeginChild("##frametimelegend", ImVec2(width, height), true)) {
    ImGui::Text("CPU Usage");
    ImGui::Separator();

    int color_idx = 0;

    ImGuiHelper::DrawLegendItem("Vsync", show_vsync_, color_idx++);
    ImGuiHelper::DrawLegendItem("Frametime", show_frametime_, color_idx++);
  }
  ImGui::EndChild();
}

void FrameTimePanel::DrawPanel(int width, int height, int time_span_seconds) {
  IMP_TRACE();

  const int selection_start_frame = data_provider_.GetSelectedFrameStart();
  const int selection_end_frame = data_provider_.GetSelectedFrameEnd();

  DrawLegend(kLegendWidth, static_cast<float>(height));
  ImGui::SameLine();  // Place plot to the right of the legend.

  // Provides a border around the plot area since we removed the padding.
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  if (ImGui::BeginChild("##FrameTimePanelChild", ImVec2(width, height),
                        ImGuiChildFlags_Borders)) {
    const float upper_bound = GetHighestVisibleFrameTimeMS(time_span_seconds);

    // Remove padding around the plot area
    ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(0, 0));

    if (ImPlot::BeginPlot("##FrameTimePanel", ImVec2(width, height),
                          ImPlotFlags_NoLegend | ImPlotFlags_NoFrame)) {
      ImPlot::SetupAxes(nullptr, nullptr,
                        ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoTickLabels,
                        ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_NoTickLabels);
      ImPlot::SetupAxisLimits(
          ImAxis_X1,
          Profiler::GetCurrentFrameIndex() -
              time_span_seconds * details::kNumDisplayValuesPerSecond,
          Profiler::GetCurrentFrameIndex(), ImPlotCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, kLowerFrameBound, upper_bound,
                              ImPlotCond_Always);

      UpdateValidTicks(upper_bound);

      if (!valid_ticks_.values.empty()) {
        ImPlot::SetupAxisTicks(ImAxis_Y1, valid_ticks_.values.data(),
                               valid_ticks_.values.size());
      }

      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

      if (!buffer_.empty()) {
        if (show_vsync_) {
          ImPlot::PlotShaded("Vsync", &buffer_.data()[0].frame_number,
                             &buffer_.data()[0].frame_time_ms,
                             buffer_.data().size(), 0, 0, buffer_.marker(),
                             sizeof(FrameTimeInfo));
        }
        if (show_frametime_) {
          ImPlot::SetNextFillStyle(ImVec4(0.0f, 1.0f, 0.0f, -1.0f), 0.5f);
          ImPlot::PlotShaded("Frametime", &buffer_.data()[0].frame_number,
                             &buffer_.data()[0].player_loop_time_ms,
                             buffer_.data().size(), 0, 0, buffer_.marker(),
                             sizeof(FrameTimeInfo));
        }

        DrawSelectedSamplePlot();

        if (ImPlot::IsPlotHovered()) {
          const ImPlotPoint mouse = ImPlot::GetPlotMousePos();
          ImDrawList* draw_list = ImPlot::GetPlotDrawList();

          const int hovered_frame = static_cast<int>(std::floor(mouse.x));

          DrawHighlightFrame(hovered_frame, draw_list);

          if (hovered_frame > 0) {
            DrawToolTip(hovered_frame);
          }

          HandleFrameSelection(hovered_frame);
        }
      }

      ImDrawList* draw_list_overlays = ImPlot::GetPlotDrawList();
      DrawHighlightFrameRange(selection_start_frame, selection_end_frame,
                              draw_list_overlays, IM_COL32(255, 255, 255, 200));
      DrawSelectedFrameLabels(selection_end_frame, draw_list_overlays);
      DrawTickLabels(draw_list_overlays, valid_ticks_);

      ImPlot::EndPlot();
    }
    ImPlot::PopStyleVar();  // ImPlotStyleVar_PlotPadding
  }
  ImGui::EndChild();
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding
}

void FrameTimePanel::UpdateValidTicks(float upper_bound) {
  valid_ticks_.clear();
  for (int i = 0; i < std::size(kTicks); ++i) {
    if (kTicks[i] <= upper_bound) {
      valid_ticks_.values.push_back(kTicks[i]);
      valid_ticks_.labels.push_back(kTickLabels[i]);
      if (valid_ticks_.values.size() >= kNumTickLabels) {
        break;
      }
    }
  }
}

void FrameTimePanel::DrawTickLabels(ImDrawList* draw_list,
                                    ValidTicks valid_ticks) {
  if (valid_ticks.values.empty()) return;

  // Manually draw Y-axis tick labels inside the plot
  ImPlot::PushPlotClipRect();
  const float plot_left_x = ImPlot::GetPlotPos().x;
  const size_t num_labels_to_draw = valid_ticks.labels.size();

  for (size_t i = 0; i < num_labels_to_draw; ++i) {
    ImVec2 label_pos =
        ImPlot::PlotToPixels(ImPlotPoint(0, valid_ticks.values[i]));
    label_pos.x = plot_left_x +
                  kTickLabelLeftPadding;  // Small padding from the left edge

    // Add a small background box for contrast.
    const ImVec2 text_size = ImGui::CalcTextSize(valid_ticks.labels[i]);
    draw_list->AddRectFilled(
        label_pos,
        ImVec2(label_pos.x + text_size.x + kTickLabelRectMarginWidth,
               label_pos.y + text_size.y + kTickLabelRectMarginHeight),
        IM_COL32(0, 0, 0, 100));

    draw_list->AddText(ImVec2(label_pos.x + kTickLabelRectMarginWidth / 2.0f,
                              label_pos.y + kTickLabelRectMarginHeight / 2.0f),
                       IM_COL32_WHITE, valid_ticks.labels[i]);
  }
  ImPlot::PopPlotClipRect();
}

void FrameTimePanel::DrawSelectedSamplePlot() {
  if (data_provider_.GetSelectedSampleName().empty()) return;
  PopulateSelectedSampleBuffer();

  ImPlot::SetNextFillStyle(ImVec4(1.0f, 0.0f, 0.0f, -1.0f), 1.0f);
  ImPlot::PlotShaded(
      "Selected Sample", &selected_sample_buffer_[0].frame_number,
      &selected_sample_buffer_[0].frame_time_ms, selected_sample_buffer_.size(),
      0, 0, 0, sizeof(SelectedSampleInfo));
}

void FrameTimePanel::PopulateSelectedSampleBuffer() {
  IMP_TRACE();

  // Don't update the buffer if we're viewing the same sample as before and
  // there are no new samples recorded.
  if (!data_provider_.HasSelectedSampleChanged() &&
      !data_provider_.WereSamplesProcessedSinceLastUpdate()) {
    return;
  }

  const std::thread::id main_thread_id = Profiler::GetMainThreadId();

  const int end_index = Profiler::GetCurrentFrameIndex() - 1;
  const int start_index = end_index - MainThreadProfilerState::kMaxFrames;

  for (size_t i = 0; i < MainThreadProfilerState::kMaxFrames; ++i) {
    const int frame_index = start_index + i;

    selected_sample_buffer_[i].frame_number = static_cast<float>(frame_index);
    selected_sample_buffer_[i].frame_time_ms = 0.0f;

    // Get all samples with the selected sample name for this frame and thread.
    const absl::StatusOr<absl::Span<SampleNode* const>> samples = GetSamples(
        data_provider_.GetSelectedSampleName(), frame_index, main_thread_id);

    // If there are no samples for this frame on this thread then we continue.
    if (!samples.ok() || samples->empty()) continue;

    uint32_t total_time_ns = 0;

    // Iterate over all samples with that name if there are any this frame.
    for (size_t j = 0; j < samples->size(); ++j) {
      total_time_ns += (*samples)[j]->total_time_ns;
    }

    selected_sample_buffer_[i].frame_time_ms =
        static_cast<float>(total_time_ns) / kNanosPerMs;
  }

  data_provider_.ClearSamplesProcessed();
  data_provider_.ClearSelectedSampleChanged();
}

absl::StatusOr<absl::Span<SampleNode* const>> FrameTimePanel::GetSamples(
    absl::string_view sample_name, int frame_index, std::thread::id thread_id) {
  if (!Profiler::HasFrameRecorded(frame_index)) {
    return absl::NotFoundError(
        absl::StrFormat("Frame %d was not recorded.", frame_index));
  }

  const ProcessedSamples& processed_frame =
      data_provider_.GetSampleProcessor().GetProcessedFrame(frame_index);

  return processed_frame.GetSamplesByName(sample_name);
}

void FrameTimePanel::DrawHighlightFrame(int frame_number, ImDrawList* draw_list,
                                        ImU32 color, float frame_width) {
  if (!draw_list) return;

  const float tool_l = ImPlot::PlotToPixels(frame_number - frame_width, 0).x;
  const float tool_r = ImPlot::PlotToPixels(frame_number + frame_width, 0).x;
  const float tool_t = ImPlot::GetPlotPos().y;
  const float tool_b = tool_t + ImPlot::GetPlotSize().y;
  ImPlot::PushPlotClipRect();
  draw_list->AddRectFilled(ImVec2(tool_l, tool_t), ImVec2(tool_r, tool_b),
                           color);
  ImPlot::PopPlotClipRect();
}

void FrameTimePanel::DrawHighlightFrameRange(int start_frame, int end_frame,
                                             ImDrawList* draw_list,
                                             ImU32 color) {
  if (!draw_list || start_frame == -1) return;

  const float tool_l = ImPlot::PlotToPixels(start_frame - 0.5f, 0).x;
  const float tool_r = ImPlot::PlotToPixels(end_frame + 0.5f, 0).x;
  const float tool_t = ImPlot::GetPlotPos().y;
  const float tool_b = tool_t + ImPlot::GetPlotSize().y;
  ImPlot::PushPlotClipRect();
  draw_list->AddRectFilled(ImVec2(tool_l, tool_t), ImVec2(tool_r, tool_b),
                           color);
  ImPlot::PopPlotClipRect();
}

void FrameTimePanel::DrawSelectedFrameLabels(int frame_number,
                                             ImDrawList* draw_list) {
  if (frame_number < 0 || buffer_.empty()) return;

  const FrameTimeInfo* frame_info = nullptr;
  for (const auto& info : buffer_.data()) {
    if (static_cast<int>(info.frame_number) == frame_number) {
      frame_info = &info;
      break;
    }
  }

  if (!frame_info) return;

  // Order must match plot order.
  int idx = 0;
  frame_value_data_[idx].y_val = frame_info->frame_time_ms;
  frame_value_data_[idx].unit = "ms";
  frame_value_data_[idx].color_index = idx;
  frame_value_data_[idx].show_flag = show_vsync_;

  idx++;
  frame_value_data_[idx].y_val = frame_info->player_loop_time_ms;
  frame_value_data_[idx].unit = "ms";
  frame_value_data_[idx].color_index = idx;
  frame_value_data_[idx].show_flag = show_frametime_;

  ImGuiHelper::DrawFrameValueLabels(frame_number, frame_value_data_, draw_list);
}

void FrameTimePanel::DrawToolTip(int frame_number) {
  if (!Profiler::HasFrameRecorded(frame_number)) return;

  ImGui::BeginTooltip();
  absl::StatusOr<uint32_t> total_frame_duration_nanos =
      Profiler::GetTotalFrameDurationNanos(frame_number);
  float frame_time_ms = 0.0f;
  if (total_frame_duration_nanos.ok()) {
    frame_time_ms = *total_frame_duration_nanos / kNanosPerMs;
  }

  absl::StatusOr<uint32_t> render_next_frame_duration_nanos =
      Profiler::GetRenderNextFrameDurationNanos(frame_number);
  float player_loop_time_ms = 0.0f;
  if (render_next_frame_duration_nanos.ok()) {
    player_loop_time_ms = *render_next_frame_duration_nanos / kNanosPerMs;
  }

  absl::StatusOr<int> sample_count = Profiler::GetSampleCount(frame_number);
  int sample_count_val = 0;
  if (sample_count.ok()) {
    sample_count_val = *sample_count;
  }

  ImGui::Text("Frame: %d", frame_number);
  ImGui::Text("Frame Time (w/ vsync): %.2fms", frame_time_ms);
  ImGui::Text("Frame Time: %.2fms", player_loop_time_ms);
  ImGui::Text("Samples: %.0d", sample_count_val);

  ImGui::EndTooltip();
}

void FrameTimePanel::Update(absl::Duration elapsed_time,
                            absl::Duration delta_time) {
  IMP_TRACE();

  // We can't use the current frame index here because that frame is still in
  // progress and the profiler can't see into the future to know how long it
  // will take. Instead we record the last frame times.
  const int64_t frame_index = Profiler::GetCurrentFrameIndex() - 1;

  absl::StatusOr<uint32_t> total_frame_duration_nanos =
      Profiler::GetTotalFrameDurationNanos(frame_index);
  float frame_time_ms = 0.0f;
  if (total_frame_duration_nanos.ok()) {
    frame_time_ms =
        static_cast<float>(*total_frame_duration_nanos) / kNanosPerMs;
  }

  absl::StatusOr<uint32_t> render_next_frame_duration_nanos =
      Profiler::GetRenderNextFrameDurationNanos(frame_index);
  float player_loop_time_ms = 0.0f;
  if (render_next_frame_duration_nanos.ok()) {
    player_loop_time_ms =
        static_cast<float>(*render_next_frame_duration_nanos) / kNanosPerMs;
  }

  buffer_.push_back(
      FrameTimeInfo{.frame_number = static_cast<float>(frame_index),
                    .frame_time_ms = frame_time_ms,
                    .player_loop_time_ms = player_loop_time_ms});
}

void FrameTimePanel::HandleFrameSelection(int hovered_frame) {
  if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    is_dragging_ = true;
    drag_start_frame_ = hovered_frame;
    data_provider_.SelectFrame(hovered_frame);
  } else if (ImGui::IsMouseDragging(ImGuiMouseButton_Left) && is_dragging_) {
    data_provider_.SelectFrames(drag_start_frame_, hovered_frame);
  } else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && is_dragging_) {
    is_dragging_ = false;
    data_provider_.SelectFrames(drag_start_frame_, hovered_frame);
    drag_start_frame_ = -1;
  } else if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
    // Right click to select single frame.
    data_provider_.SelectFrame(hovered_frame);
  }
}

}  // namespace imp::editor
