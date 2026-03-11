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

#include <array>
#include <thread>  // NOLINT: Need to sort things by thread id.
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "dear_imgui/imgui.h"
#include "core/editor/widgets/performance/circular_buffer.h"
#include "core/editor/widgets/performance/flame_graph.h"
#include "core/editor/widgets/performance/hierarchy_panel.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/performance/profiler.h"
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

  absl::string_view GetSelectedSampleName() const {
    return selected_sample_name_;
  }

  void SetSelectedSampleName(absl::string_view selected_sample_name) {
    selected_sample_name_ = selected_sample_name;
    selected_sample_changed_ = true;
  }

 private:
  // An internal class for structuring data for ImPlot to draw. ImPlot can only
  // draw data on axes of the same type, so everything here has to be stored as
  // floats.
  struct FrameTimeInfo {
    // The number of frames that has been recorded
    float frame_number;
    // Total time spent in the frame.
    float frame_time_ms;
    // Time spent in FilamentHost::RenderNextFrame() this frame.
    float player_loop_time_ms;
  };

  struct SelectedSampleInfo {
    // The number of frames that has been recorded
    float frame_number;
    // Total time spent in the frame.
    float frame_time_ms;
  };

  struct ValidTicks {
    std::vector<double> values;
    std::vector<const char*> labels;
    void clear() {
      values.clear();
      labels.clear();
    }
  };

  enum class ProfilerDetailsViewMode { kHierarchy, kFlameGraph };

  void DrawLegend(float width, float height);
  // Returns the highest frame time that is visible in the plot.
  float GetHighestVisibleFrameTimeMS(int time_span_seconds);
  // Draws a grey highlight over the frame being moused over
  void DrawHighlightFrame(int frame_number, ImDrawList* draw_list,
                          ImU32 color = IM_COL32(128, 128, 128, 64),
                          float frame_width = 0.5f);
  // Draws a tool tip showing more information on the frame
  void DrawToolTip(int frame_number);
  // Populates the selected sample buffer with the total time spent in the
  // selected sample for each frame in the buffer.
  void PopulateSelectedSampleBuffer();
  // Draws a separate plot representing the frame time of the selected sample.
  void DrawSelectedSamplePlot();
  // Updates the valid ticks based on the highest frame time in the plot.
  void UpdateValidTicks(float upper_bound);
  // Draws the tick labels based on the valid ticks.
  void DrawTickLabels(ImDrawList* draw_list, ValidTicks valid_ticks);
  // Draws the options to swap between hierarchy/flame graph views.
  void DrawOptionsBar();
  std::vector<SampleNode*>* GetSamples(absl::string_view sample_name,
                                       int frame_index,
                                       std::thread::id thread_id);
  BaseView& view_;
  CircularBuffer<FrameTimeInfo> buffer_;
  std::array<SelectedSampleInfo, Profiler::kMaxFrames> selected_sample_buffer_;

  // This is incremented on every frame that is updated on this panel. This is
  // different from the actual frame number Impress is at.
  int frame_number_;
  MonitorState state_ = MonitorState::kRunning;

  int selected_frame_number_ = -1;
  ViewConfig view_config_;
  HierarchyPanel hierarchy_panel_;
  FlameGraph flame_graph_;
  SampleProcessor sample_processor_;
  absl::string_view selected_sample_name_ = "";
  bool samples_processed_since_last_update_ = false;
  bool show_vsync_ = true;
  bool show_frametime_ = true;
  bool selected_sample_changed_ = false;
  ValidTicks valid_ticks_;

  // Which view to display (hierarchy or flame graph)
  ProfilerDetailsViewMode profiler_details_view_mode_ =
      ProfilerDetailsViewMode::kHierarchy;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_FRAME_TIME_PANEL_H_
