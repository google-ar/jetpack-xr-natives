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

#include <cmath>
#include <cstdint>
#include <vector>

#include "absl/time/time.h"
#include "dear_imgui/imgui.h"
#include "implot/implot.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/config.h"
#include "core/editor/widgets/performance/hierarchy_panel.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/performance/profiler.h"
#include "core/view/base_view.h"

namespace imp::editor {

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

void FrameTimePanel::DrawPanel(int width, int height, int time_span_seconds) {
  IMP_TRACE();

  constexpr float lower_bound = 0;
  float upper_bound = GetHighestVisibleFrameTimeMS(time_span_seconds);

  if (ImPlot::BeginPlot("##FrameTimePanel", ImVec2(width, height))) {
    ImPlot::SetupAxes("Frame number", "Frame time (ms)", ImPlotAxisFlags_Lock,
                      ImPlotAxisFlags_Lock);
    ImPlot::SetupAxisLimits(
        ImAxis_X1,
        Profiler::GetCurrentFrameIndex() -
            time_span_seconds * details::kNumDisplayValuesPerSecond,
        Profiler::GetCurrentFrameIndex(), ImPlotCond_Always);
    ImPlot::SetupAxisLimits(ImAxis_Y1, lower_bound, upper_bound,
                            ImPlotCond_Always);
    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

    if (!buffer_.empty()) {
      ImPlot::PlotShaded("Vsync", &buffer_.data()[0].frame_number,
                         &buffer_.data()[0].frame_time_ms,
                         buffer_.data().size(), 0, 0, buffer_.marker(),
                         sizeof(FrameTimeInfo));

      ImPlot::SetNextFillStyle(ImVec4(0.0f, 1.0f, 0.0f, -1.0f), 0.5f);
      ImPlot::PlotShaded("Frametime", &buffer_.data()[0].frame_number,
                         &buffer_.data()[0].player_loop_time_ms,
                         buffer_.data().size(), 0, 0, buffer_.marker(),
                         sizeof(FrameTimeInfo));

      if (ImPlot::IsPlotHovered()) {
        ImDrawList* draw_list = ImPlot::GetPlotDrawList();
        ImPlotPoint mouse = ImPlot::GetPlotMousePos();

        int hovered_frame = static_cast<int>(std::floor(mouse.x));

        DrawHighlightFrame(hovered_frame, draw_list);

        if (hovered_frame > 0) {
          selected_frame_number_ = hovered_frame;

          DrawToolTip(hovered_frame);
        }
      }
    }

    ImPlot::EndPlot();
  }

  // If the panel is not paused, show the most recent frame recorded.
  if (state_ != MonitorState::kPaused)
    selected_frame_number_ = Profiler::GetCurrentFrameIndex() - 1;

  ShowHierarchyPanel(selected_frame_number_);
}

void FrameTimePanel::ShowHierarchyPanel(int frame_number) {
  hierarchy_panel_.DrawPanel(frame_number);
}

void FrameTimePanel::DrawHighlightFrame(int frame_number,
                                        ImDrawList* draw_list) {
  if (!draw_list) {
    return;
  }

  float tool_l = ImPlot::PlotToPixels(frame_number - 0.5f, 0).x;
  float tool_r = ImPlot::PlotToPixels(frame_number + 0.5f, 0).x;
  float tool_t = ImPlot::GetPlotPos().y;
  float tool_b = tool_t + ImPlot::GetPlotSize().y;
  ImPlot::PushPlotClipRect();
  draw_list->AddRectFilled(ImVec2(tool_l, tool_t), ImVec2(tool_r, tool_b),
                           IM_COL32(128, 128, 128, 64));
  ImPlot::PopPlotClipRect();
}

void FrameTimePanel::DrawToolTip(int frame_number) {
  if (!Profiler::HasFrameRecorded(frame_number)) return;

  ImGui::BeginTooltip();
  float frame_time_ms =
      Profiler::GetTotalFrameDurationNanos(frame_number) / 1000000.0f;
  float player_loop_time_ms =
      Profiler::GetRenderNextFrameDurationNanos(frame_number) / 1000000.0f;

  ImGui::Text("Frame Time (w/ vsync): %.2fms", frame_time_ms);
  ImGui::Text("Frame Time: %.2fms", player_loop_time_ms);

  ImGui::EndTooltip();
}

void FrameTimePanel::Update(absl::Duration elapsed_time,
                            absl::Duration delta_time) {
  IMP_TRACE();

  // We can't use the current frame index here because that frame is still in
  // progress and the profiler can't see into the future to know how long it
  // will take. Instead we record the last frame times.
  int64_t frame_index = Profiler::GetCurrentFrameIndex() - 1;
  float frame_time_ms =
      static_cast<float>(Profiler::GetTotalFrameDurationNanos(frame_index)) /
      1000000.0f;
  float player_loop_time_ms =
      static_cast<float>(
          Profiler::GetRenderNextFrameDurationNanos(frame_index)) /
      1000000.0f;
  buffer_.push_back(
      FrameTimeInfo{.frame_number = static_cast<float>(frame_index),
                    .frame_time_ms = frame_time_ms,
                    .player_loop_time_ms = player_loop_time_ms});
}

}  // namespace imp::editor
