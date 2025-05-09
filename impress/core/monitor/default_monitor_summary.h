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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_DEFAULT_MONITOR_SUMMARY_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_DEFAULT_MONITOR_SUMMARY_H_

#include <cstdint>
#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/monitor/monitor.h"
#include "core/monitor/monitor_summary.h"
#include "core/monitor/value_measurement.h"
#include "core/ncsb/update_system.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp {

//
// Reports internal stats periodically by configuring a MonitorSummary.

// Samples are kept separately in 3 moving windows using a weighted exponential
// moving average.  The average age of the samples in each bucket is printed out
// in the log along with any values which are being tracked.

// Usage:
//   In your app instantiate and it works automatically:
//   View->GetRegistry().GetOrCreate<imp::DefaultMonitorSummary>(*View);

// SideEffects:
//   The MonitorSummary periodically clears data in the measurements it is
// tracking.  If the measurement is being used elsewhere totals may be reset.
// Output:
//   Logged approximately every minute, one line with contents like this:
//   "avg sample age:  60 / 180 / 420 (seconds), ms per frame: 16.74 / 16.70
// / 16.69, percent on schedule: 99.59 / 99.79 / 99.88, index buffers: 210.88 /
// 226.19 / 243.03, vertex buffers: 210.88 / 226.19 / 243.03, materials: 13.08
// / 13.26 / 13.55, textures: 249.83 / 225.62 / 189.92"
//
class DefaultMonitorSummary
    : public UpdateSystem::Updater<DefaultMonitorSummary> {
 public:
  explicit DefaultMonitorSummary(BaseView& view);

  // Periodically resamples the measurements.
  // Periodically outputs the summary to the log.
  void Update(const FrameTime& frame_time) override;

  MonitorSummary& GetSummary() { return summary_; }

  // The number of frames between outputting the summary to log.
  // If 0 the summary will not be output.
  int64_t FramesBetweenLogging() const { return frames_between_output_; }
  void SetFramesBetweenLogging(int64_t frames_between_output) {
    frames_between_output_ = frames_between_output;
  }

  // Adds a summary of frames past the deadline.
  // maximum_display_period will be truncated to the millisecond.
  // Setting Zero duration will remove the metric.
  // Calling multiple times will replace previous metrics.
  void SetDisplayPeriod(absl::Duration maximum_display_period,
                        absl::string_view histogram_name);

  absl::Duration GetDisplayPeriod() const { return display_period_; }

 private:
  BaseView& view_;
  Monitor& monitor_;
  MonitorSummary summary_;
  int64_t frames_between_output_ = 3000;
  int64_t frames_between_sampling_ = 90;
  absl::Duration display_period_;
  std::optional<MonitorSummary::CustomMetricHandle> display_period_metric_;
  ValueMeasurement bufferObjectCount_;
  ValueMeasurement viewCount_;
  ValueMeasurement sceneCount_;
  ValueMeasurement swapChainCount_;
  ValueMeasurement streamCount_;
  ValueMeasurement indexBufferCount_;
  ValueMeasurement skinningBufferCount_;
  ValueMeasurement morphTargetBufferCount_;
  ValueMeasurement instanceBufferCount_;
  ValueMeasurement vertexBufferCount_;
  ValueMeasurement indirectLightCount_;
  ValueMeasurement materialCount_;
  ValueMeasurement textureCount_;
  ValueMeasurement skyboxCount_;
  ValueMeasurement colorGradingCount_;
  ValueMeasurement renderTargetCount_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_DEFAULT_MONITOR_SUMMARY_H_
