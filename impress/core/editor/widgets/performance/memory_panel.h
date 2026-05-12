/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_MEMORY_PANEL_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_MEMORY_PANEL_H_

#include <array>
#include <cstddef>

#include "absl/time/time.h"
#include "dear_imgui/imgui.h"
#include "core/editor/widgets/performance/circular_buffer.h"
#include "core/editor/widgets/performance/imgui_helper.h"
#include "core/editor/widgets/performance/monitor_panel.h"

namespace imp::editor {

class MemoryPanel : public MonitorPanel {
 public:
  MemoryPanel(int buffer_size);
  ~MemoryPanel() override;

  void DrawPlot(int current_frame, int oldest_frame);
  void DrawPanel(int width, int height, int time_span_seconds) override;
  void Update(absl::Duration elapsed_time, absl::Duration delta_time) override;
  void OnStateChanged(MonitorState state) override;

 private:
  static constexpr size_t kKilobyte = 1024;
  static constexpr size_t kMegabyte = 1024 * 1024;
  float GetHighestVisibleMemoryUsage(int time_span_seconds);
  void DrawLegend(int width, int height);
  // Draws a grey highlight over the frame being moused over
  void DrawHighlightFrame(int frame_number, ImDrawList* draw_list,
                          ImU32 color = IM_COL32(128, 128, 128, 64),
                          float size = 0.5f);
  // Draws a tool tip showing more information on the number of renderables in
  // the frame.
  void DrawToolTip(int frame_number);
  // Draws labels for the selected frame.
  void DrawSelectedFrameLabels(int frame_number, ImDrawList* draw_list);

  bool show_allocations_ = true;
  bool show_memory_ = true;

  struct MemoryInfo {
    float frame_number;
    float allocated_megabytes;
    float allocations;
  };

  CircularBuffer<MemoryInfo> buffer_;
  int upper_bound_ = 0;
  MonitorState state_ = MonitorState::kRunning;

  static constexpr int kNumFrameValueLabels = 2;
  std::array<ImGuiHelper::LabelData, kNumFrameValueLabels> frame_value_data_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_MEMORY_PANEL_H_
