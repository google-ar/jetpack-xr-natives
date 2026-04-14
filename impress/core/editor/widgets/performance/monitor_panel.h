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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_MONITOR_PANEL_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_MONITOR_PANEL_H_

#include "absl/time/time.h"

namespace imp::editor {

// A panel for displaying performance monitoring information.
struct MonitorPanel {
  // Draws the panel for displaying the performance data. The width and height
  // is provided for proper displaying of the panel in the performance tab.
  //
  // This is called whenever ImGui renders.
  //
  // The suggested implementation of DrawPanel is:
  //
  // DrawPanel(int width, int height, int time_span_seconds) {
  //   if (ImPlot::BeginPlot(<Name of panel>, ImVec2(width, height))) {
  //     // Plotting performance data here
  //     ImPlot::EndPlot();
  // }
  virtual void DrawPanel(int width, int height, int time_span_seconds) = 0;

  // Update this panel with the latest frame timings.
  virtual void Update(absl::Duration elapsed_time,
                      absl::Duration delta_time) = 0;

  virtual ~MonitorPanel() = default;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_MONITOR_PANEL_H_
