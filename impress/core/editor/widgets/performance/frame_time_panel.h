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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_FRAME_TIME_PANEL_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_FRAME_TIME_PANEL_H_

#include "absl/time/time.h"
#include "dear_imgui/imgui.h"
#include "core/editor/widgets/performance/circular_buffer.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/view/base_view.h"
#include "core/view/utils/proto/view_config.proto.imp.h"

namespace imp::editor {

// A panel for the performance monitor to show the various information on the
// lifetime of a frame
class FrameTimePanel : public MonitorPanel {
 public:
  FrameTimePanel(BaseView& view, int buffer_size);
  ~FrameTimePanel() override;

  void DrawPanel(int width, int height, int time_span_seconds) override;
  void Update(absl::Duration elapsed_time, absl::Duration delta_time) override;
  void OnStateChanged(MonitorState state) override;

 private:
  // An internal class for structuring data for ImPlot to draw. ImPlot can only
  // draw data on axes of the same type, so everything here has to be stored as
  // floats.
  struct FrameTimeInfo {
    // The number of frames that has been recorded
    float frame_number;
    // Stores the total elapsed time of the performance monitor
    float elapsed_time_ms;
    // The view advance/update time
    float advance_time_ms;
    // The entire time taken by Filament to render the scene
    float render_time_ms;
    // The overall frame time
    float frame_time_ms;
    // The time the foreground executor took this frame
    float foreground_executor_time_ms;
  };

  // Draws a grey highlight over the frame being moused over
  void DrawHighlightFrame(int frame_number, ImDrawList* draw_list);
  // Draws a tool tip showing more information on the frame
  void DrawToolTip(int frame_number);
  BaseView& view_;
  CircularBuffer<FrameTimeInfo> buffer_;
  // This is incremented on every frame that is updated on this panel. This is
  // different from the actual frame number Impress is at.
  int frame_number_;
  MonitorState state_ = MonitorState::kRunning;

  ViewConfig view_config_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_FRAME_TIME_PANEL_H_
