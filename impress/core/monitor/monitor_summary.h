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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_MONITOR_LOGGER_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_MONITOR_LOGGER_H_

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "core/math/vec.h"
#include "core/monitor/duration_measurement_data.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/monitor.h"
#include "core/monitor/value_measurement_data.h"
#include "core/monitor/windowed_average.h"
#include "robin_map/include/tsl/robin_set.h"

namespace imp {
// Default update per collection.
constexpr int kDefaultUpdatesPerSampleCollection = 300;

// The sizes below control the midpoint sample for each
// window. For example if set to 10, then the first 10 samples have as much
// weight as all of the older samples combined.
constexpr double kFirstWindowSize = 6;
constexpr double kSecondWindowSize = 24;
constexpr double kThirdWindowSize = 60;
const double kFirstWindowWeight = exp(-1.0 / kFirstWindowSize);
const double kSecondWindowWeight = exp(-1.0 / kSecondWindowSize);
const double kThirdWindowWeight = exp(-1.0 / kThirdWindowSize);

// Returns a value to represent the highest values captured in the histogram.
std::optional<uint64_t> GetLastBucketLowerBound(
    Monitor& monitor, absl::string_view metric_with_histogram_data_name);

// Extracts data and trends from the monitor and reports them as a string.
//
// Data collection is destructive. Measurements used will be reset to zero.
// Custom Metrics can be added, by adding an instance of CustomMetric.
// Details at (broken link)
class MonitorSummary {
 public:
  explicit MonitorSummary(Monitor& monitor) : monitor_(monitor) {}

  struct WindowedAverageFactory {
    WindowedAverage CreateWindowedAverage(double initial_value) {
      return WindowedAverage(initial_value, first_window_weight,
                             second_window_weight, third_window_weight);
    }
    void SetWindowedAverageWeights(double first_weight, double second_weight,
                                   double third_weight) {
      first_window_weight = first_weight;
      second_window_weight = second_weight;
      third_window_weight = third_weight;
    }
    double first_window_weight = kFirstWindowWeight;
    double second_window_weight = kSecondWindowWeight;
    double third_window_weight = kThirdWindowWeight;
  } windowed_average_factory_;

  using CustomMetricHandle = uint64_t;

  // Abstract class for each field in the logger.
  class CustomMetric {
   public:
    CustomMetric() : handle_(next_handle_++) {}
    virtual void Update() = 0;
    virtual std::string ToString() const = 0;

    template <typename Sink>
    friend void AbslStringify(Sink& sink, const CustomMetric& p) {
      sink.Append(p.ToString());
    }

    virtual ~CustomMetric() = default;

    std::optional<float3> GetAverages() const {
      if (!average_) {
        return std::nullopt;
      }
      return average_->GetAverages();
    }

    CustomMetricHandle GetHandle() const { return handle_; }

    void Reset() { average_.reset(); }

   protected:
    std::optional<WindowedAverage> average_;

   private:
    CustomMetricHandle handle_;
    static CustomMetricHandle next_handle_;
  };

  // Printable via absl string library.
  template <typename Sink>
  friend void AbslStringify(Sink& sink, const MonitorSummary& p) {
    std::string accumulated_result;
    std::string single_result;
    for (auto& metric : p.metrics_) {
      single_result = metric->ToString();
      if (single_result.empty()) {
        continue;
      }

      if (!accumulated_result.empty()) {
        absl::StrAppend(&accumulated_result, ", ");
      }
      absl::StrAppend(&accumulated_result, single_result);
    }
    sink.Append(accumulated_result);
  }

  // After updates_per_collection_ calls to Update() the data will be
  // collected and the measurements will be reset.
  void Update();

  // Add a metric to the logger.
  template <typename T, typename... Args>
  CustomMetricHandle AddMetric(absl::string_view prefix, Args... args) {
    metrics_.emplace_back(std::make_unique<T>(*this, monitor_, prefix,
                                              std::forward<Args>(args)...));
    return metrics_.back()->GetHandle();
  }

  // Add a user defined metric to the logger.
  inline MonitorSummary::CustomMetricHandle AddCustomMetric(
      std::unique_ptr<CustomMetric> metric) {
    metrics_.push_back(std::move(metric));
    return metrics_.back()->GetHandle();
  }

  inline void AddMeasurementData(MeasurementData* measurement_data) {
    measurement_data_.insert(measurement_data);
  }

  // Configures the Monitor Summary and the MovingAverage weights.
  // The average reported by each window is the average of the samples in the
  // window. The weights are used to determine the midpoint sample for each
  // window while putting more weight on recent samples.
  // Parameters:
  // updates_per_second: The number of updates per second (if the updates of the
  // MonitorSummary are driven by the frame rate, this should be set to the
  // frame rate).
  // seconds_in_first_average: The number of seconds covered by samples in the
  // first MovingAverage window.
  // seconds_in_second_average: The number of seconds covered by samples in the
  // second MovingAverage window.
  // seconds_in_third_average: The number of seconds covered by samples in the
  // third MovingAverage window.
  void Configure(int updates_per_sample_collection,
                 double updates_per_second = 30,
                 double seconds_in_first_average = 60,
                 double seconds_in_second_average = 300,
                 double seconds_in_third_average = 900) {
    updates_per_sample_collection_ = updates_per_sample_collection;
    double seconds_per_collection =
        updates_per_sample_collection_ / updates_per_second;
    double first_window_size =
        seconds_in_first_average / seconds_per_collection;
    double second_window_size =
        seconds_in_second_average / seconds_per_collection;
    double third_window_size =
        seconds_in_third_average / seconds_per_collection;
    windowed_average_factory_.SetWindowedAverageWeights(
        exp(-1.0 / first_window_size), exp(-1.0 / second_window_size),
        exp(-1.0 / third_window_size));
  }

  // Exposed for testing.
  const std::vector<std::unique_ptr<CustomMetric>>& GetMetrics() const {
    return metrics_;
  }

  absl::Status RemoveCustomMetric(
      MonitorSummary::CustomMetricHandle metric_handle);

 private:
  Monitor& monitor_;
  // updated every frame, until updates_per_collection_ is reached.
  int64_t update_count_ = 0;
  // The number of frames between samples.
  int64_t updates_per_sample_collection_ = kDefaultUpdatesPerSampleCollection;
  std::vector<std::unique_ptr<CustomMetric>> metrics_;
  tsl::robin_set<MeasurementData*> measurement_data_;
};

// Helper class to calculate and report avg duration / count.
class PeriodInMsMetric : public MonitorSummary::CustomMetric {
 public:
  PeriodInMsMetric(MonitorSummary& monitor_summary, Monitor& monitor,
                   absl::string_view prefix, absl::string_view duration,
                   absl::string_view count);

  std::string ToString() const override;

  void Update() override;

  virtual ~PeriodInMsMetric() = default;

 private:
  MonitorSummary::WindowedAverageFactory window_average_factory_;
  absl::string_view prefix_;
  DurationMeasurementData* duration_data_;
  DurationMeasurementData* count_data_;
};

// Reports (duration(interval_end)/sample count(interval_end) -
// duration(interval_start)/sample count(interval_start))
class IntervalInMsMetric : public MonitorSummary::CustomMetric {
 public:
  IntervalInMsMetric(MonitorSummary& monitor_summary, Monitor& monitor,
                     absl::string_view prefix, absl::string_view interval_start,
                     absl::string_view interval_end);
  std::string ToString() const;
  void Update() override;

 private:
  MonitorSummary::WindowedAverageFactory window_average_factory_;
  absl::string_view prefix_;
  DurationMeasurementData* interval_start_data_;
  DurationMeasurementData* interval_end_data_;
};

// Reports 100 * (duration(interval_end)/sample count(interval_end) -
// duration(interval_start)/sample count(interval_start)) /
// (duration(denominator)/sample count(denominator)).
class PercentageOfIntervalMetric : public MonitorSummary::CustomMetric {
 public:
  PercentageOfIntervalMetric(MonitorSummary& monitor_summary, Monitor& monitor,
                             absl::string_view prefix,
                             absl::string_view numerator_start,
                             absl::string_view numerator_end,
                             absl::string_view denominator);
  void Update() override;
  std::string ToString() const override;

 private:
  MonitorSummary::WindowedAverageFactory window_average_factory_;
  absl::string_view prefix_;
  DurationMeasurementData* numerator_start_data_;
  DurationMeasurementData* numerator_end_data_;
  DurationMeasurementData* denominator_data_;
};

// Reports 100 * duration(numerator)/duration(denominator)
class PercentageOfDurationMetric : public MonitorSummary::CustomMetric {
 public:
  PercentageOfDurationMetric(MonitorSummary& monitor_summary, Monitor& monitor,
                             absl::string_view prefix,
                             absl::string_view numerator,
                             absl::string_view denominator);
  std::string ToString() const override;
  void Update() override;

 private:
  MonitorSummary::WindowedAverageFactory window_average_factory_;
  absl::string_view prefix_;
  DurationMeasurementData* numerator_data_;
  DurationMeasurementData* denominator_data_;
};

// Reports 100 * count(numerator)/count(denominator)
class PercentageOfCountMetric : public MonitorSummary::CustomMetric {
 public:
  PercentageOfCountMetric(MonitorSummary& monitor_summary, Monitor& monitor,
                          absl::string_view prefix, absl::string_view numerator,
                          absl::string_view denominator);

  std::string ToString() const override;
  void Update() override;

 private:
  MonitorSummary::WindowedAverageFactory window_average_factory_;
  absl::string_view prefix_;
  DurationMeasurementData* numerator_data_;
  DurationMeasurementData* denominator_data_;
};

// Report the number of values from a range of values as a percentage of the
// total number of histogram values. If lower_bound or upper_bound is not set,
// that side of the histogram is unbounded.
// Returns the CustomMetric for later removal.
class PercentageOfHistogramMetric : public MonitorSummary::CustomMetric {
 public:
  PercentageOfHistogramMetric(MonitorSummary& monitor_summary, Monitor& monitor,
                              absl::string_view prefix,
                              absl::string_view metric_with_histogram,
                              std::optional<uint64_t> lower_bound_ms,
                              std::optional<uint64_t> upper_bound_ms);

  PercentageOfHistogramMetric(MonitorSummary& monitor_summary, Monitor& monitor,
                              absl::string_view prefix,
                              absl::string_view metric_with_histogram);

  std::string ToString() const override;
  void Update() override;

 private:
  MonitorSummary::WindowedAverageFactory window_average_factory_;
  absl::string_view prefix_;
  DurationMeasurementData* metric_with_histogram_data_;
  std::optional<uint64_t> lower_bound_ms_;
  std::optional<uint64_t> upper_bound_ms_;
};

// Averages a ValueMeasurement stored in the monitor.
// ValueMeasurements will only be logged if the observed value is changing.
class ValueMetric : public MonitorSummary::CustomMetric {
 public:
  ValueMetric(MonitorSummary& monitor_summary, Monitor& monitor,
              absl::string_view prefix, absl::string_view metric_with_value);

  void Update() override;
  std::string ToString() const override;

 private:
  MonitorSummary::WindowedAverageFactory window_average_factory_;
  absl::string_view prefix_;
  ValueMeasurementData* value_measurement_data_;
};

// Captures a timestamp on Update() and reports the average the timestamps.
class SampleAgeMetric : public MonitorSummary::CustomMetric {
 public:
  SampleAgeMetric(MonitorSummary& monitor_summary, Monitor& monitor,
                  absl::string_view prefix);

  void Update() override;
  std::string ToString() const override;

 private:
  MonitorSummary::WindowedAverageFactory window_average_factory_;
  absl::string_view prefix_;
  Monitor& monitor_;
};

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_MONITOR_LOGGER_H_
