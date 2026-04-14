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
#include "core/editor/widgets/performance/callstack_panel.h"
#include "core/editor/widgets/performance/circular_buffer.h"
#include "core/editor/widgets/performance/flame_graph.h"
#include "core/editor/widgets/performance/hierarchy_panel.h"
#include "core/editor/widgets/performance/imgui_helper.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/performance/profiler.h"
#include "core/view/base_view.h"

namespace imp::editor {

class PerformanceWindow;

// A panel for the performance monitor to show the various information on the
// lifetime of a frame
class FrameTimePanel : public MonitorPanel {
 public:
  FrameTimePanel(PerformanceWindow& performance_window, BaseView& view,
                 int buffer_size);
  ~FrameTimePanel() override;

  void DrawPanel(int width, int height, int time_span_seconds) override;
  void Update(absl::Duration elapsed_time, absl::Duration delta_time) override;

  absl::string_view GetSelectedSampleName() const {
    return selected_sample_name_;
  }

  void SetSelectedSampleName(absl::string_view selected_sample_name) {
    selected_sample_name_ = selected_sample_name;
    selected_sample_changed_ = true;
  }
  bool IsCallstackPanelEnabled() const { return show_callstack_ != 0; }
  bool IsDragging() const { return is_dragging_; }

 private:
  // Number of labels to draw next to the vertical line on the plot.
  static constexpr int kNumFrameValueLabels = 2;

  // Width of the call stack panel when it is first opened.
  static constexpr float kCallstackPanelStartingWidth = 450.0f;

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

  // Draws the legend to the left of the frame time plot.
  void DrawLegend(float width, float height);

  // Returns the highest frame time that is visible in the plot.
  float GetHighestVisibleFrameTimeMS(int time_span_seconds);

  // Draws a grey highlight over the frame being moused over
  void DrawHighlightFrame(int frame_number, ImDrawList* draw_list,
                          ImU32 color = IM_COL32(128, 128, 128, 64),
                          float frame_width = 0.5f);

  // Draws a grey highlight over the selected frame range.
  void DrawHighlightFrameRange(int start_frame, int end_frame,
                               ImDrawList* draw_list, ImU32 color);

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
  // Draws the splitter between the sample view and the call stack view.
  void DrawSplitter();

  // Draws a label next to each plot showing its value at the selected frame.
  void DrawSelectedFrameLabels(int frame_number, ImDrawList* draw_list);

  // Returns all samples with a specific name for a given frame and thread.
  const std::vector<SampleNode*>* GetSamples(absl::string_view sample_name,
                                             int frame_index,
                                             std::thread::id thread_id);

  PerformanceWindow& performance_window_;
  BaseView& view_;
  CircularBuffer<FrameTimeInfo> buffer_;
  std::array<SelectedSampleInfo, Profiler::kMaxFrames> selected_sample_buffer_;

  // This is incremented on every frame that is updated on this panel. This is
  // different from the actual frame number Impress is at.
  int frame_number_;

  ViewConfig view_config_;
  HierarchyPanel hierarchy_panel_;
  CallstackPanel callstack_panel_;
  FlameGraph flame_graph_;
  SampleProcessor sample_processor_;
  absl::string_view selected_sample_name_ = "";
  bool samples_processed_since_last_update_ = false;
  bool show_vsync_ = true;
  bool show_frametime_ = true;
  bool selected_sample_changed_ = false;
  int show_callstack_ = 0;
  float sample_view_width_ = -1.0f;
  float callstack_panel_width_ = kCallstackPanelStartingWidth;
  ValidTicks valid_ticks_;
  std::array<ImGuiHelper::LabelData, kNumFrameValueLabels> frame_value_data_;

  // Which view to display (hierarchy or flame graph)
  ProfilerDetailsViewMode profiler_details_view_mode_ =
      ProfilerDetailsViewMode::kHierarchy;

  // Frame selection.
  void HandleFrameSelection(int hovered_frame);
  bool is_dragging_ = false;
  int drag_start_frame_ = -1;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_FRAME_TIME_PANEL_H_
