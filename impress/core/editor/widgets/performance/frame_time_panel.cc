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

#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "dear_imgui/imgui.h"
#include "implot/implot.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/editor/widgets/performance/config.h"
#include "core/editor/widgets/performance/hierarchy_panel.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/monitor/duration_measurement_data.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/monitor.h"
#include "core/monitor/monitor_helpers.h"
#include "core/performance/profiler.h"
#include "core/view/base_view.h"

namespace imp::editor {

namespace {

absl::Duration GetLatestDurationMeasurement(Monitor* monitor,
                                            absl::string_view measurement_id) {
  MeasurementData::MeasurementId id = monitor->GetMeasurementId(measurement_id);
  DurationMeasurementData* data =
      static_cast<DurationMeasurementData*>(monitor->GetMeasurementData(id));
  return data->GetLatestSampleDuration();
}

}  // namespace

FrameTimePanel::FrameTimePanel(BaseView& view, int buffer_size)
    : view_(view), buffer_(buffer_size), view_config_(view.GetConfig()) {}

FrameTimePanel::~FrameTimePanel() = default;

void FrameTimePanel::OnStateChanged(MonitorPanel::MonitorState state) {
  state_ = state;
}

void FrameTimePanel::DrawPanel(int width, int height, int time_span_seconds) {
  IMP_TRACE_NAME("FrameTimePanel::DrawPanel");
  constexpr float upper_bound = 60;
  constexpr float lower_bound = 0;

  ImPlotCond plot_cond =
      state_ == MonitorState::kPaused ? ImPlotCond_None : ImPlotCond_Always;

  if (ImPlot::BeginPlot("##FrameTimePanel", ImVec2(width, height))) {
    ImPlot::SetupAxes("Frame number", "Frame time (ms)");
    ImPlot::SetupAxisLimits(
        ImAxis_X1,
        Profiler::GetCurrentFrameIndex() -
            time_span_seconds * details::kNumDisplayValuesPerSecond,
        Profiler::GetCurrentFrameIndex(), plot_cond);
    ImPlot::SetupAxisLimits(ImAxis_Y1, lower_bound, upper_bound);
    ImPlot::SetupAxisLimitsConstraints(ImAxis_Y1, 0, 1000);
    ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

    if (!buffer_.empty()) {
      ImPlot::PlotBars("Frame time", &buffer_.data()[0].frame_number,
                       &buffer_.data()[0].frame_time_ms, buffer_.data().size(),
                       1, 0, buffer_.marker(), sizeof(FrameTimeInfo));

      ImPlot::PlotBars("Filament render time", &buffer_.data()[0].frame_number,
                       &buffer_.data()[0].render_time_ms, buffer_.data().size(),
                       1, 0, buffer_.marker(), sizeof(FrameTimeInfo));

      ImPlot::PlotBars("View Advance time", &buffer_.data()[0].frame_number,
                       &buffer_.data()[0].advance_time_ms,
                       buffer_.data().size(), 1, 0, buffer_.marker(),
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
#if !IMP_PLATFORM(WASM)
  if (selected_frame_number_ > 0) ShowHierarchyPanel(selected_frame_number_);
#endif
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
  const FrameTimeInfo& info =
      buffer_.data()[(frame_number - 1) % buffer_.capacity()];
  ImVec4 warning_color = ImVec4(1.0f, 0.25f, 0.25f, 1.0f);

  ImGui::BeginTooltip();
  ImGui::Text("Elapsed Time:  %.2fms", info.elapsed_time_ms);
  ImGui::Text("Frame Time: %.2fms", info.frame_time_ms);
  ImGui::Text("Filament Render Time:   %.2fms", info.render_time_ms);
  ImGui::Text("Advance Time:  %.2fms", info.advance_time_ms);

  if (info.foreground_executor_time_ms >
      view_config_.foreground_executor_timeout_ms) {
    ImGui::TextColored(warning_color, "Foreground Executor Time:  %.2fms",
                       info.foreground_executor_time_ms);
  } else {
    ImGui::Text("Foreground Executor Time:  %.2fms",
                info.foreground_executor_time_ms);
  }

  ImGui::EndTooltip();
}

void FrameTimePanel::Update(absl::Duration elapsed_time,
                            absl::Duration delta_time) {
  IMP_TRACE_NAME("FrameTimePanel::Update");
  // TODO Ensure that all values shown are correct and in sync
  float view_frame_time = absl::ToDoubleMilliseconds(
      GetLatestDurationMeasurement(view_.GetMonitor(), kFramePresented));
  float filament_render_time = absl::ToDoubleMilliseconds(
      GetLatestDurationMeasurement(view_.GetMonitor(), kFilamentFrameTiming));
  float view_advance_time = absl::ToDoubleMilliseconds(
      GetLatestDurationMeasurement(view_.GetMonitor(), kViewAdvance));
  float foreground_executor_time =
      absl::ToDoubleMilliseconds(GetLatestDurationMeasurement(
          view_.GetMonitor(), kForegroundExecutorTiming));

  float elapsed_time_ms = absl::ToDoubleMilliseconds(elapsed_time);

  buffer_.push_back(FrameTimeInfo{
      .frame_number = static_cast<float>(Profiler::GetCurrentFrameIndex()),
      .elapsed_time_ms = elapsed_time_ms,
      .advance_time_ms = view_advance_time,
      .render_time_ms = filament_render_time,
      .frame_time_ms = view_frame_time,
      .foreground_executor_time_ms = foreground_executor_time});
}

}  // namespace imp::editor
