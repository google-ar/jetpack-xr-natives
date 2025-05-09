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

#include "absl/time/time.h"
#include "dear_imgui/imgui.h"
#include "core/editor/widgets/performance/circular_buffer.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/view/base_view.h"

namespace imp::editor {

// A panel for the performance monitor to show the various information related
// to rendering
class RenderInfoPanel : public MonitorPanel {
 public:
  RenderInfoPanel(BaseView& view, int buffer_size);
  ~RenderInfoPanel() override;

  void DrawPanel(int width, int height, int time_span_seconds) override;
  void Update(absl::Duration elapsed_time, absl::Duration delta_time) override;
  void OnStateChanged(MonitorState state) override;

 private:
  // Draws a grey highlight over the frame being moused over
  void DrawHighlightFrame(int frame_number, ImDrawList* draw_list);
  // Draws a tool tip showing more information on the number of renderables in
  // the frame.
  void DrawToolTip(int frame_number);
  struct RenderInfo {
    int frame_number;
    int num_renderables;
    int num_sprites;
    int num_gltfs;
  };

  bool has_sprites_ = false;
  bool has_gltfs_ = false;

  BaseView& view_;
  CircularBuffer<RenderInfo> buffer_;
  int frame_number_ = 0;
  int upper_bound_ = 0;
  MonitorState state_ = MonitorState::kRunning;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_RENDER_INFO_PANEL_H_
