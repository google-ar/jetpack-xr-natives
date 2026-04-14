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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_RENDER_INFO_PANEL_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_RENDER_INFO_PANEL_H_

#include <array>

#include "absl/time/time.h"
#include "dear_imgui/imgui.h"
#include "core/editor/widgets/performance/circular_buffer.h"
#include "core/editor/widgets/performance/imgui_helper.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/view/base_view.h"

namespace imp::editor {

class PerformanceWindow;

// A panel for the performance monitor to show the various information related
// to rendering
class RenderInfoPanel : public MonitorPanel {
 public:
  RenderInfoPanel(PerformanceWindow& performance_window, BaseView& view,
                  int buffer_size);
  ~RenderInfoPanel() override;

  static constexpr ImU32 kDefaultHighlightColor = IM_COL32(128, 128, 128, 64);
  void DrawPanel(int width, int height, int time_span_seconds) override;
  void Update(absl::Duration elapsed_time, absl::Duration delta_time) override;

 private:
  // Draws a grey highlight over the frame being moused over
  void DrawHighlightFrame(int frame_number, ImDrawList* draw_list,
                          ImU32 color = kDefaultHighlightColor,
                          float size = 0.5f);
  // Draws a tool tip showing more information on the number of renderables in
  // the frame.
  void DrawToolTip(int frame_number);
  void DrawSelectedFrameLabels(int frame_number, ImDrawList* draw_list);
  struct RenderInfo {
    int frame_number;
    int num_renderables;
    int num_sprites;
    int num_gltfs;
  };

  bool has_sprites_ = false;
  bool has_gltfs_ = false;

  void DrawLegend(float width, float height);
  bool show_renderables_ = true;
  bool show_sprites_ = true;
  bool show_gltfs_ = true;

  PerformanceWindow& performance_window_;
  BaseView& view_;
  CircularBuffer<RenderInfo> buffer_;
  int upper_bound_ = 0;

  static constexpr int kNumFrameValueLabels = 3;
  std::array<ImGuiHelper::LabelData, kNumFrameValueLabels> frame_value_data_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_RENDER_INFO_PANEL_H_
