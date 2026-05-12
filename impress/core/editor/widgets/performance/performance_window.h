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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_PERFORMANCE_WINDOW_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_PERFORMANCE_WINDOW_H_

#include <memory>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/update_system.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

// Window at the bottom of the screen showing Impress info, warning, and error
// logs. Useful for identifying problems with the model parsing and loading.
class PerformanceWindow : public Widget, public imp::Rememberer {
 public:
  explicit PerformanceWindow(BaseView& view);
  ~PerformanceWindow() override;

  void DrawImGui() override;

  void Update(const FrameTime& frame_time);

  absl::string_view GetName() const override { return "Performance"; }

  void AddPanel(std::unique_ptr<MonitorPanel> monitor_panel);

 private:
  void OnViewPostRender();
  void DrawMonitorPanels();


  BaseView& view_;
  std::vector<std::unique_ptr<MonitorPanel>> monitor_panels_;
  float time_span_seconds_;
  MonitorPanel::MonitorState monitor_state_ =
      MonitorPanel::MonitorState::kRunning;
  Dispatcher::ScopedConnection post_frame_connection_;
  int selected_frame_number_ = 0;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_PERFORMANCE_WINDOW_H_
