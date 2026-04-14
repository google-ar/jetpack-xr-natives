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

#include "core/editor/widgets/performance/memory_panel.h"

#include <cmath>
#include <vector>

#include "absl/time/time.h"
#include "dear_imgui/imgui.h"
#include "implot/implot.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/config.h"
#include "core/editor/widgets/performance/imgui_helper.h"
#include "core/editor/widgets/performance/performance_window.h"
#include "core/performance/profiler.h"

namespace {
// Bytes in a megabyte.
constexpr int kBytesPerMegabyte = 1024 * 1024;

// Width of the legend to the left of the plot.
constexpr int kLegendWidth = 150;

// Lower bound for the memory usage plot.
constexpr float kLowerBound = 0.0f;

// Padding for the upper bound of the memory usage plot.
constexpr float kUpperBoundMargin = 1.1f;

// Color of the selected frame line in the plot.
constexpr ImU32 kSelectedFrameColor = IM_COL32(255, 255, 255, 200);

// Width of the selected frame line in the plot.
constexpr float kHighlightFrameWidth = 0.3f;
}  // namespace

namespace imp::editor {

MemoryPanel::MemoryPanel(PerformanceWindow& performance_window, int buffer_size)
    : performance_window_(performance_window), buffer_(buffer_size) {}

MemoryPanel::~MemoryPanel() = default;

float MemoryPanel::GetHighestVisibleMemoryUsage(int time_span_seconds) {
  const int earliest_visible_frame =
      Profiler::GetCurrentFrameIndex() -
      time_span_seconds * details::kNumDisplayValuesPerSecond;

  float highest_frame_memory_allocated = 0.0f;

  for (const auto& memory_info : buffer_.data()) {
    if (memory_info.frame_number < earliest_visible_frame) continue;

    // Use whichever value is higher as our upper bound.
    // Ensures no matter what that both plots will be visible.
    if (memory_info.allocated_megabytes >= highest_frame_memory_allocated) {
      highest_frame_memory_allocated = memory_info.allocated_megabytes;
    } else if (memory_info.allocations > highest_frame_memory_allocated) {
      highest_frame_memory_allocated = memory_info.allocations;
    }
  }
  return highest_frame_memory_allocated;
}

void MemoryPanel::DrawLegend(const int width, const int height) {
  if (ImGui::BeginChild("##memorylegend", ImVec2(width, height), true)) {
    ImGui::Text("Memory Usage");
    ImGui::Separator();

    int color_idx = 0;

    ImGuiHelper::DrawLegendItem("Memory Used", show_memory_, color_idx++);
    ImGuiHelper::DrawLegendItem("Allocations", show_allocations_, color_idx++);
  }
  ImGui::EndChild();
}

void MemoryPanel::DrawPanel(const int width, const int height,
                            const int time_span_seconds) {
  IMP_TRACE();

  DrawLegend(kLegendWidth, height);
  ImGui::SameLine();  // Place plot to the right of the legend.

  // Provides a border around the plot area since we removed the padding.
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  if (ImGui::BeginChild("##MemoryInfoChild", ImVec2(width, height), true)) {
    // Remove padding around the plot area
    ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(0, 0));

    const int current_frame = Profiler::GetCurrentFrameIndex();
    const int oldest_frame =
        current_frame -
        details::kMaxTimeSpanSeconds * details::kNumDisplayValuesPerSecond;
    const int oldest_shown_frame =
        current_frame - time_span_seconds * details::kNumDisplayValuesPerSecond;
    const float upper_bound =
        GetHighestVisibleMemoryUsage(time_span_seconds) * kUpperBoundMargin;

    if (ImPlot::BeginPlot("##Memory Info", ImVec2(width, height),
                          ImPlotFlags_NoLegend | ImPlotFlags_NoFrame)) {
      ImPlot::SetupAxes(nullptr, nullptr,
                        ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoTickLabels,
                        ImPlotAxisFlags_Lock | ImPlotAxisFlags_NoTickLabels);
      ImPlot::SetupAxisLimits(ImAxis_X1, oldest_shown_frame, current_frame,
                              ImPlotCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, kLowerBound, upper_bound,
                              ImPlotCond_Always);

      ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);

      DrawPlot(current_frame, oldest_frame);

      const int selected_frame_number =
          performance_window_.GetSelectedFrameNumber();
      ImDrawList* draw_list_overlays = ImPlot::GetPlotDrawList();

      DrawHighlightFrame(selected_frame_number, draw_list_overlays,
                         kSelectedFrameColor, kHighlightFrameWidth);
      DrawSelectedFrameLabels(selected_frame_number, draw_list_overlays);

      ImPlot::EndPlot();
    }
    ImPlot::PopStyleVar();  // ImPlotStyleVar_PlotPadding
  }

  ImGui::EndChild();
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding
}

void MemoryPanel::DrawPlot(const int current_frame, const int oldest_frame) {
  if (buffer_.empty()) return;

  if (show_memory_) {
    ImPlot::SetAxis(ImAxis_Y1);
    ImPlot::PlotLine("Allocated Bytes", &buffer_.data()[0].frame_number,
                     &buffer_.data()[0].allocated_megabytes,
                     buffer_.data().size(), 0, buffer_.marker(),
                     sizeof(MemoryInfo));
  }

  if (show_allocations_) {
    ImPlot::PlotLine("Allocations", &buffer_.data()[0].frame_number,
                     &buffer_.data()[0].allocations, buffer_.data().size(), 0,
                     buffer_.marker(), sizeof(MemoryInfo));
  }

  if (ImPlot::IsPlotHovered()) {
    ImDrawList* draw_list = ImPlot::GetPlotDrawList();
    const ImPlotPoint mouse = ImPlot::GetPlotMousePos();

    const int hovered_frame = static_cast<int>(std::floor(mouse.x));

    if (hovered_frame < current_frame && hovered_frame >= oldest_frame) {
      DrawHighlightFrame(hovered_frame, draw_list);
      DrawToolTip(hovered_frame);
    }
  }
}

void MemoryPanel::DrawHighlightFrame(const int frame_number,
                                     ImDrawList* draw_list, const ImU32 color,
                                     const float size) {
  if (!draw_list) return;

  const float tool_l = ImPlot::PlotToPixels(frame_number - size, 0).x;
  const float tool_r = ImPlot::PlotToPixels(frame_number + size, 0).x;
  const float tool_t = ImPlot::GetPlotPos().y;
  const float tool_b = tool_t + ImPlot::GetPlotSize().y;
  ImPlot::PushPlotClipRect();
  draw_list->AddRectFilled(ImVec2(tool_l, tool_t), ImVec2(tool_r, tool_b),
                           color);
  ImPlot::PopPlotClipRect();
}

void MemoryPanel::DrawSelectedFrameLabels(int frame_number,
                                          ImDrawList* draw_list) {
  if (frame_number < 0 || buffer_.empty()) return;

  const MemoryInfo* frame_info = nullptr;
  for (const auto& info : buffer_.data()) {
    if (static_cast<int>(info.frame_number) == frame_number) {
      frame_info = &info;
      break;
    }
  }

  if (!frame_info) return;

  // Order must match plot order.
  int idx = 0;
  frame_value_data_[idx].y_val = frame_info->allocated_megabytes;
  frame_value_data_[idx].unit = "MB";
  frame_value_data_[idx].color_index = idx;
  frame_value_data_[idx].show_flag = show_memory_;

  idx++;
  frame_value_data_[idx].y_val = frame_info->allocations;
  frame_value_data_[idx].unit = "k";
  frame_value_data_[idx].color_index = idx;
  frame_value_data_[idx].show_flag = show_allocations_;

  ImGuiHelper::DrawFrameValueLabels(frame_number, frame_value_data_, draw_list);
}

void MemoryPanel::DrawToolTip(int frame_number) {
  if (buffer_.empty()) return;

  int found_idx = -1;
  const std::vector<MemoryInfo>& data = buffer_.data();

  {
    IMP_TRACE_NAME("MemoryPanel::DrawToolTip - Binary Search");
    int size = buffer_.size();
    int capacity = buffer_.capacity();
    int marker = buffer_.marker();

    // Binary search on the circular buffer
    int low = 0;
    int high = size - 1;

    while (low <= high) {
      int mid = low + (high - low) / 2;
      // Calculate the actual index in the underlying vector
      int actual_mid_idx = (marker + mid) % capacity;
      if (size < capacity) {
        actual_mid_idx = mid;
      }

      const MemoryInfo& current = data[actual_mid_idx];

      if (current.frame_number == frame_number) {
        found_idx = actual_mid_idx;
        break;
      } else if (current.frame_number < frame_number) {
        low = mid + 1;
      } else {
        high = mid - 1;
      }
    }
  }

  if (found_idx == -1) return;
  const MemoryInfo& info = data[found_idx];

  ImGui::BeginTooltip();

  ImGui::Text("Total Memory Allocated: %.1f MB", info.allocated_megabytes);
  ImGui::Text("Total Allocations: %.0f k", info.allocations);
  ImGui::Text("Frame Number: %.0f", info.frame_number);

  ImGui::EndTooltip();
}

void MemoryPanel::Update(absl::Duration elapsed_time,
                         absl::Duration delta_time) {
  static constexpr double kBytesPerMegabyteDouble =
      static_cast<double>(kBytesPerMegabyte);

  buffer_.push_back(MemoryInfo{
      .frame_number = static_cast<float>(Profiler::GetCurrentFrameIndex()),
      .allocated_megabytes = static_cast<float>(
          Profiler::GetMemoryUsageBytes() / kBytesPerMegabyteDouble),
      .allocations =
          static_cast<float>(Profiler::GetAllocationsCountTotal() / 1000.0)});
}

}  // namespace imp::editor
