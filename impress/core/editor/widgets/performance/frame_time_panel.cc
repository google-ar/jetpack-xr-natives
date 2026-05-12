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

#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "dear_imgui/imgui.h"
#include "implot/implot.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/circular_buffer.h"
#include "core/editor/widgets/performance/config.h"
#include "core/editor/widgets/performance/hierarchy_panel.h"
#include "core/editor/widgets/performance/imgui_helper.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/performance/profiler.h"
#include "core/view/base_view.h"

namespace imp::editor {

namespace {
// Custom Y-axis ticks for common frame rate targets
static constexpr double kTicks[] = {
    1000.0 / 30.0,   // 30 FPS
    1000.0 / 60.0,   // 60 FPS
    1000.0 / 90.0,   // 90 FPS
    1000.0 / 120.0,  // 120 FPS
    1000.0 / 144.0,  // 144 FPS
    1000.0 / 240.0   // 240 FPS
};

static constexpr const char* kTickLabels[] = {
    "33.3ms (30fps)", "16.7ms (60fps)", "11.1ms (90fps)",
    "8.1ms (120fps)", "6.9ms (144fps)", "4.2ms (240fps)"};

// How many tick labels to display at once.
// Running at 30fps you will see the tick labels for 60, 90, and 30fps.
// At 120 you will see the labels for 120, 144, and 240fps.
static constexpr int kNumTickLabels = 3;

// Nanoseconds in a millisecond.
static constexpr float kNanosPerMs = 1000000.0f;
}  // namespace

FrameTimePanel::FrameTimePanel(BaseView& view, int buffer_size)
    : view_(view), buffer_(buffer_size), view_config_(view.GetConfig()) {}

FrameTimePanel::~FrameTimePanel() = default;

void FrameTimePanel::OnStateChanged(MonitorPanel::MonitorState state) {
  state_ = state;
}

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

  constexpr float kLegendWidth = 150.0f;
  const int selected_frame_number = ImGuiHelper::GetSelectedFrameNumber();

  DrawLegend(kLegendWidth, height);
  ImGui::SameLine();  // Place plot to the right of the legend.

  // Provides a border around the plot area since we removed the padding.
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  if (ImGui::BeginChild("##FrameTimePanelChild", ImVec2(width, height),
                        ImGuiChildFlags_Borders)) {
    constexpr float lower_bound = 0;
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
      ImPlot::SetupAxisLimits(ImAxis_Y1, lower_bound, upper_bound,
                              ImPlotCond_Always);

      UpdateValidTicks(upper_bound);

      if (!valid_ticks_.values.empty()) {
        ImPlot::SetupAxisTicks(ImAxis_Y1, valid_ticks_.values.data(),
                               kNumTickLabels);
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
          ImPlotPoint mouse = ImPlot::GetPlotMousePos();
          ImDrawList* draw_list = ImPlot::GetPlotDrawList();

          int hovered_frame = static_cast<int>(std::floor(mouse.x));

          DrawHighlightFrame(hovered_frame, draw_list);

          if (hovered_frame > 0) {
            DrawToolTip(hovered_frame);
          }

          if (ImGui::IsMouseDown(ImPlot::GetInputMap().SelectCancel)) {
            ImGuiHelper::SelectFrame(hovered_frame);
          }
        }
      }

      ImDrawList* draw_list_overlays = ImPlot::GetPlotDrawList();
      DrawHighlightFrame(selected_frame_number, draw_list_overlays,
                         IM_COL32(255, 255, 255, 200), 0.3f);
      DrawSelectedFrameLabels(selected_frame_number, draw_list_overlays);
      DrawTickLabels(draw_list_overlays, valid_ticks_);

      ImPlot::EndPlot();
    }
    ImPlot::PopStyleVar();  // ImPlotStyleVar_PlotPadding
  }
  ImGui::EndChild();
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding

  // Options for changing the view and thread to display samples for.
  DrawOptionsBar(selected_frame_number);

  if (profiler_details_view_mode_ == ProfilerDetailsViewMode::kHierarchy) {
    hierarchy_panel_.DrawPanel(selected_frame_number, sample_processor_, *this);
  } else {
    flame_graph_.DrawPanel(selected_frame_number, sample_processor_, *this);
  }
}

void FrameTimePanel::DrawOptionsBar(int selected_frame_number) {
  // Toggle to switch between Hierarchy and Flame Graph.
  if (ImGui::RadioButton(
          "Hierarchy",
          profiler_details_view_mode_ == ProfilerDetailsViewMode::kHierarchy)) {
    profiler_details_view_mode_ = ProfilerDetailsViewMode::kHierarchy;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Flame Graph",
                         profiler_details_view_mode_ ==
                             ProfilerDetailsViewMode::kFlameGraph)) {
    profiler_details_view_mode_ = ProfilerDetailsViewMode::kFlameGraph;
  }
}

void FrameTimePanel::UpdateValidTicks(float upper_bound) {
  valid_ticks_.clear();
  for (int i = 0; i < std::size(kTicks); ++i) {
    if (kTicks[i] <= upper_bound) {
      valid_ticks_.values.push_back(kTicks[i]);
      valid_ticks_.labels.push_back(kTickLabels[i]);
    }
  }
}

void FrameTimePanel::DrawTickLabels(ImDrawList* draw_list,
                                    ValidTicks valid_ticks) {
  constexpr float kTickLabelLeftPadding = 20.0f;
  constexpr float kTickLabelRectMarginWidth = 4.0f;
  constexpr float kTickLabelRectMarginHeight = 2.0f;
  constexpr float kTickLabelTextOffsetX = 2.0f;
  constexpr float kTickLabelTextOffsetY = 1.0f;

  if (valid_ticks.values.empty()) return;

  // Manually draw Y-axis tick labels inside the plot
  ImPlot::PushPlotClipRect();
  float plot_left_x = ImPlot::GetPlotPos().x;
  size_t num_labels_to_draw =
      std::min(static_cast<size_t>(kNumTickLabels), valid_ticks.labels.size());

  for (size_t i = 0; i < num_labels_to_draw; ++i) {
    ImVec2 label_pos =
        ImPlot::PlotToPixels(ImPlotPoint(0, valid_ticks.values[i]));
    label_pos.x = plot_left_x +
                  kTickLabelLeftPadding;  // Small padding from the left edge

    // Add a small background box for contrast.
    ImVec2 text_size = ImGui::CalcTextSize(valid_ticks.labels[i]);
    draw_list->AddRectFilled(
        label_pos,
        ImVec2(label_pos.x + text_size.x + kTickLabelRectMarginWidth,
               label_pos.y + text_size.y + kTickLabelRectMarginHeight),
        IM_COL32(0, 0, 0, 100));

    draw_list->AddText(ImVec2(label_pos.x + kTickLabelTextOffsetX,
                              label_pos.y + kTickLabelTextOffsetY),
                       IM_COL32_WHITE, valid_ticks.labels[i]);
  }
  ImPlot::PopPlotClipRect();
}

void FrameTimePanel::DrawSelectedSamplePlot() {
  if (selected_sample_name_.empty()) return;
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
  if (!selected_sample_changed_ && !samples_processed_since_last_update_) {
    return;
  }

  std::thread::id main_thread_id = Profiler::GetMainThreadId();

  int end_index = Profiler::GetCurrentFrameIndex() - 1;
  int start_index = end_index - Profiler::kMaxFrames;

  for (size_t i = 0; i < Profiler::kMaxFrames; ++i) {
    int frame_index = start_index + i;

    selected_sample_buffer_[i].frame_number = static_cast<float>(frame_index);
    selected_sample_buffer_[i].frame_time_ms = 0.0f;

    // Get all samples with the selected sample name for this frame and thread.
    std::vector<SampleNode*>* samples_ptr =
        GetSamples(selected_sample_name_, frame_index, main_thread_id);

    // If there are no samples for this frame on this thread then we continue.
    if (!samples_ptr) continue;

    std::vector<SampleNode*>& samples = *samples_ptr;

    uint32_t total_time_ns = 0;

    // Iterate over all samples with that name if there are any this frame.
    for (size_t j = 0; j < samples.size(); ++j) {
      total_time_ns += samples[j]->total_time_ns;
    }

    selected_sample_buffer_[i].frame_time_ms =
        static_cast<float>(total_time_ns) / kNanosPerMs;
  }

  samples_processed_since_last_update_ = false;
  selected_sample_changed_ = false;
}

std::vector<SampleNode*>* FrameTimePanel::GetSamples(
    absl::string_view sample_name, int frame_index, std::thread::id thread_id) {
  ProcessedSamples& processed_frame =
      sample_processor_.GetProcessedFrame(frame_index);

  const auto& sample_it = processed_frame.samples_by_name.find(sample_name);

  if (sample_it == processed_frame.samples_by_name.end()) return nullptr;

  return &sample_it->second;
}

void FrameTimePanel::DrawHighlightFrame(int frame_number, ImDrawList* draw_list,
                                        ImU32 color, float frame_width) {
  if (!draw_list) return;

  float tool_l = ImPlot::PlotToPixels(frame_number - frame_width, 0).x;
  float tool_r = ImPlot::PlotToPixels(frame_number + frame_width, 0).x;
  float tool_t = ImPlot::GetPlotPos().y;
  float tool_b = tool_t + ImPlot::GetPlotSize().y;
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
  float frame_time_ms =
      Profiler::GetTotalFrameDurationNanos(frame_number) / kNanosPerMs;
  float player_loop_time_ms =
      Profiler::GetRenderNextFrameDurationNanos(frame_number) / kNanosPerMs;

  ImGui::Text("Frame: %d", frame_number);
  ImGui::Text("Frame Time (w/ vsync): %.2fms", frame_time_ms);
  ImGui::Text("Frame Time: %.2fms", player_loop_time_ms);
  ImGui::Text("Samples: %.0d", Profiler::GetSampleCount(frame_number));

  ImGui::EndTooltip();
}

void FrameTimePanel::Update(absl::Duration elapsed_time,
                            absl::Duration delta_time) {
  IMP_TRACE();

  // We can't use the current frame index here because that frame is still in
  // progress and the profiler can't see into the future to know how long it
  // will take. Instead we record the last frame times.
  int64_t frame_index = Profiler::GetCurrentFrameIndex() - 1;
  sample_processor_.ProcessMainThreadSamples(frame_index);
  samples_processed_since_last_update_ = true;
  float frame_time_ms =
      static_cast<float>(Profiler::GetTotalFrameDurationNanos(frame_index)) /
      kNanosPerMs;
  float player_loop_time_ms =
      static_cast<float>(
          Profiler::GetRenderNextFrameDurationNanos(frame_index)) /
      kNanosPerMs;
  buffer_.push_back(
      FrameTimeInfo{.frame_number = static_cast<float>(frame_index),
                    .frame_time_ms = frame_time_ms,
                    .player_loop_time_ms = player_loop_time_ms});
}

}  // namespace imp::editor
