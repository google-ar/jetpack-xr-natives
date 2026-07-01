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
#include <string>
#include <thread>  // NOLINT: Need to sort by thread id.
#include <vector>

#include "absl/strings/string_view.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/editor/widgets/performance/monitor_panel.h"
#include "core/editor/widgets/performance/profiler_data_provider.h"
#include "core/editor/widgets/performance/sample_processor.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/update_system.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

class ProfilerDetailsSection;

// Window at the bottom of the screen showing Impress info, warning, and error
// logs. Useful for identifying problems with the model parsing and loading.
class PerformanceWindow : public Widget,
                          public imp::Rememberer,
                          public ProfilerDataProvider {
 public:
  enum MonitorState {
    kRunning,
    kPaused,
  };

  explicit PerformanceWindow(BaseView& view);
  ~PerformanceWindow() override;

  void DrawImGui() override;

  void Update(const FrameTime& frame_time);

  absl::string_view GetName() const override { return "Performance"; }

  void AddPanel(std::unique_ptr<MonitorPanel> monitor_panel);

  void SelectFrame(int frame_number) override;
  void SelectFrames(int start_frame, int end_frame) override;

  int GetSelectedFrameStart() const override { return selected_frame_start_; }
  int GetSelectedFrameEnd() const override { return selected_frame_end_; }
  int GetSelectedFrameNumber() const override { return selected_frame_end_; }

  absl::string_view GetSelectedSampleName() const override {
    return selected_sample_name_;
  }
  void SetSelectedSampleName(absl::string_view name) override;

  std::thread::id GetSelectedSampleThreadId() const override {
    return selected_sample_thread_id_;
  }
  void SetSelectedSampleThreadId(std::thread::id thread_id) override;

  SampleProcessor& GetSampleProcessor() override { return sample_processor_; }
  bool WereSamplesProcessedSinceLastUpdate() const override {
    return samples_processed_since_last_update_;
  }
  void ClearSamplesProcessed() override {
    samples_processed_since_last_update_ = false;
  }
  bool HasSelectedSampleChanged() const override {
    return selected_sample_changed_;
  }
  void ClearSelectedSampleChanged() override {
    selected_sample_changed_ = false;
  }

  MonitorState GetMonitorState() const { return monitor_state_; }
  void SetMonitorState(MonitorState monitor_state);

 private:
  void OnViewPostRender();
  void DrawMonitorPanels();
  void DrawSplitter();

  BaseView& view_;
  std::vector<std::unique_ptr<MonitorPanel>> monitor_panels_;
  std::unique_ptr<ProfilerDetailsSection> details_panel_;

  float time_span_seconds_;
  Dispatcher::ScopedConnection post_frame_connection_;
  int selected_frame_start_ = -1;
  int selected_frame_end_ = -1;

  std::string selected_sample_name_ = "";
  std::thread::id selected_sample_thread_id_;
  bool selected_sample_changed_ = false;
  bool samples_processed_since_last_update_ = false;
  SampleProcessor sample_processor_;

  MonitorState monitor_state_ = MonitorState::kPaused;

  float graphs_height_ = -1.0f;
  float details_height_ = -1.0f;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_PERFORMANCE_WINDOW_H_
