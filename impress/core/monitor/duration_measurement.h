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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_DURATION_MEASUREMENT_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_DURATION_MEASUREMENT_H_

#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/monitor.h"

namespace imp {

// Interval based measurement of time.
class DurationMeasurement {
 public:
  // Construction does not take ownership of the monitor, but uses it.
  // The Monitor must be valid for the lifetime of this class.
  DurationMeasurement(Monitor* monitor, absl::string_view name);
  ~DurationMeasurement() = default;

  // Configures a histogram for a Duration Measurement.
  static void AddHistogram(Monitor& monitor, absl::string_view name,
                           absl::Duration lower_bound,
                           absl::Duration bucket_width, int bucket_count);

  // If true, will report moving_avg_time within
  // ::logs::proto::vr::arcore::impress::DurationReport.
  // The default value is false.
  void SetEnableAverages(bool enable_averages);

  // If true, will report percentile0_time and percentile100_time within
  // ::logs::proto::vr::arcore::impress::DurationReport.
  // The default value is false.
  void SetEnablePercentiles(bool enable_percentiles);

  // Time is recorded from BeginSample to EndSample
  void BeginSample();

  // Ends time counting and accumulates results.
  void EndSample();

  // Ends time counting and increments cancelled sample count without
  // accumulating results.
  void CancelSample();

  // Ends time counting without incrementing anything.
  void DiscardSample();

  // Adds a specific amount of time as a sample.
  void AddSample(absl::Duration sample);

  // Returns true if BeginSample has been called without a matching EndSample
  bool IsInProgress() const;

 private:
  Monitor* monitor_;
  MeasurementData::MeasurementId id_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_DURATION_MEASUREMENT_H_
