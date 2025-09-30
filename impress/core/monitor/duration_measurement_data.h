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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_DURATION_MEASUREMENT_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_DURATION_MEASUREMENT_DATA_H_

#include <stdint.h>

#include <optional>
#include <string>

#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/common/typed_id.h"
#include "core/math/moving_average.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/simple_histogram.h"

namespace imp {

class Monitor;

// Datastore for duration based measurements.
// By default the histogram is not calculated.
// Call AddHistogram if a histogram report is desired.
class DurationMeasurementData : public MeasurementData {
 public:
  explicit DurationMeasurementData(Monitor* monitor, absl::string_view name);
  ~DurationMeasurementData() override;

  // Configures a histogram for a Duration Measurement.
  static void AddHistogram(Monitor& monitor, absl::string_view name,
                           absl::Duration lower_bound,
                           absl::Duration bucket_width, int bucket_count);

  // Starts counting time for a duration.
  void BeginSample();

  // Ends time counting and accumulates results.
  void EndSample();

  // Ends time counting and increments cancelled sample count without
  // accumulating results.
  void CancelSample();

  // Ends time counting and does not count as either a sample or a "cancelled"
  // sample.
  void DiscardSample();

  // Records a new sample.
  void AddSample(absl::Duration sample);

  // If true, will report moving_avg_time within
  // ::logs::proto::vr::arcore::impress::DurationReport.
  // The default value is false.
  void SetEnableAverages(bool enable_averages) {
    enable_averages_ = enable_averages;
  }

  // If true, will report percentile0_time and percentile100_time within
  // ::logs::proto::vr::arcore::impress::DurationReport.
  // The default value is false.
  void SetEnablePercentiles(bool enable_percentiles) {
    enable_percentiles_ = enable_percentiles;
  }

  bool GetEnableAverages() const { return enable_averages_; }
  bool GetEnablePercentiles() const { return enable_percentiles_; }
  uint64_t GetSampleCount() const;

  absl::Duration GetTotalDuration() const;

  const absl::Duration& GetShortestSampleDuration() const {
    return shortest_sample_duration_;
  }

  const absl::Duration& GetLongestSampleDuration() const {
    return longest_sample_duration_;
  }

  absl::Duration GetLatestSampleDuration() const { return latest_sample_; }
  absl::Duration GetMovingAverageSampleDuration() const {
    return absl::Milliseconds(average_sample_duration_ms_.GetAverage());
  }

  uint64_t GetCancelledSampleCount() const;

  const absl::optional<SimpleHistogram>& GetHistogram() const {
    return histogram_;
  }

  absl::string_view GetName() const override;

  void Reset() override;

  bool IsInProgress() const;

 private:
  // Allows samples to be tracked in a histogram. By default they are not
  // tracked. bucket_count must be above zero and bucket width must be non zero
  // lower_bound to enable the histogram. The valid range is [lower_bound,
  // bucket_width*bucket_count).
  void InternalAddHistogram(absl::Duration lower_bound,
                            absl::Duration bucket_width, int bucket_count);

  Monitor* monitor_;
  std::string name_;
  bool in_progress_;
  absl::Time start_time_;
  absl::Time end_time_;

  absl::Duration latest_sample_;
  uint64_t sample_count_;
  absl::Duration total_duration_;
  absl::Duration shortest_sample_duration_;
  absl::Duration longest_sample_duration_;
  MovingAverage average_sample_duration_ms_;
  absl::optional<SimpleHistogram> histogram_;

  uint64_t cancelled_sample_count_;

  bool enable_averages_ = false;
  bool enable_percentiles_ = false;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_DURATION_MEASUREMENT_DATA_H_
