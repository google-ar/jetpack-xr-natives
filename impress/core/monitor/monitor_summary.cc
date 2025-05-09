// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/monitor/monitor_summary.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "absl/types/variant.h"
#include "core/math/vec.h"
#include "core/monitor/duration_measurement.h"
#include "core/monitor/duration_measurement_data.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/monitor.h"
#include "core/monitor/simple_histogram.h"
#include "core/monitor/value_measurement_data.h"

namespace imp {
namespace {

DurationMeasurementData* GetOrAddDurationMeasurementData(
    MonitorSummary& logger, Monitor& monitor, absl::string_view name) {
  // Create the measurement if needed
  DurationMeasurement(&monitor, name);
  MeasurementData* data =
      monitor.GetMeasurementData(monitor.GetMeasurementId(name));
  logger.AddMeasurementData(data);
  return static_cast<DurationMeasurementData*>(data);
}

}  // namespace

MonitorSummary::CustomMetricHandle MonitorSummary::CustomMetric::next_handle_ =
    0;

// Returns a value to represent the highest values captured in the histogram.
std::optional<uint64_t> GetLastBucketLowerBound(
    Monitor& monitor, absl::string_view metric_with_histogram_data_name) {
  DurationMeasurement(&monitor, metric_with_histogram_data_name);
  DurationMeasurementData* metric_with_histogram_data =
      static_cast<DurationMeasurementData*>(monitor.GetMeasurementData(
          monitor.GetMeasurementId(metric_with_histogram_data_name)));
  if (!metric_with_histogram_data->GetHistogram().has_value()) {
    return std::nullopt;
  }
  absl::Span<const SimpleHistogram::Bucket> buckets =
      metric_with_histogram_data->GetHistogram().value().GetBuckets();
  if (buckets.empty()) {
    return std::nullopt;
  }
  SimpleHistogram::Bucket last_bucket = buckets.back();
  return last_bucket.bucket_lower_bound;
}

void MonitorSummary::Update() {
  update_count_++;
  if (update_count_ % updates_per_sample_collection_ != 0) {
    return;
  }
  for (auto& metric : metrics_) {
    metric->Update();
  }
  for (auto& measurement_data : measurement_data_) {
    measurement_data->Reset();
  }
}

absl::Status MonitorSummary::RemoveCustomMetric(
    MonitorSummary::CustomMetricHandle metric) {
  for (auto it = metrics_.begin(); it != metrics_.end(); ++it) {
    if ((*it)->GetHandle() == metric) {
      metrics_.erase(it);
      return absl::OkStatus();
    }
  }
  return absl::NotFoundError("Metric not found");
}

PeriodInMsMetric::PeriodInMsMetric(MonitorSummary& monitor_summary,
                                   Monitor& monitor, absl::string_view prefix,
                                   absl::string_view duration,
                                   absl::string_view count)
    : window_average_factory_(monitor_summary.windowed_average_factory_),
      prefix_(prefix),
      duration_data_(
          GetOrAddDurationMeasurementData(monitor_summary, monitor, duration)),
      count_data_(
          GetOrAddDurationMeasurementData(monitor_summary, monitor, count)) {}

std::string PeriodInMsMetric::ToString() const {
  if (!average_) {
    return std::string{};
  }
  return absl::StrCat(prefix_, *average_);
}

void PeriodInMsMetric::Update() {
  double period_in_ms = 0;
  if (count_data_->GetSampleCount() > 0 &&
      duration_data_->GetTotalDuration() >= absl::ZeroDuration()) {
    period_in_ms =
        absl::ToDoubleMilliseconds(duration_data_->GetTotalDuration()) /
        count_data_->GetSampleCount();
    if (!average_) {
      average_ = window_average_factory_.CreateWindowedAverage(period_in_ms);
    } else {
    }
    average_->AddSample(period_in_ms);
  }
}

IntervalInMsMetric::IntervalInMsMetric(MonitorSummary& monitor_summary,
                                       Monitor& monitor,
                                       absl::string_view prefix,
                                       absl::string_view interval_start,
                                       absl::string_view interval_end)
    : window_average_factory_(monitor_summary.windowed_average_factory_),
      prefix_(prefix),
      interval_start_data_(GetOrAddDurationMeasurementData(
          monitor_summary, monitor, interval_start)),
      interval_end_data_(GetOrAddDurationMeasurementData(
          monitor_summary, monitor, interval_end)) {}

std::string IntervalInMsMetric::ToString() const {
  if (!average_) {
    return std::string{};
  }
  return absl::StrCat(prefix_, *average_);
}
void IntervalInMsMetric::Update() {
  if (interval_start_data_->GetSampleCount() < 1 ||
      interval_end_data_->GetSampleCount() < 1) {
    return;
  }

  double interval_start_period_in_ms =
      absl::ToDoubleMilliseconds(interval_start_data_->GetTotalDuration()) /
      interval_start_data_->GetSampleCount();
  double interval_end_period_in_ms =
      absl::ToDoubleMilliseconds(interval_end_data_->GetTotalDuration()) /
      interval_end_data_->GetSampleCount();

  if (interval_start_period_in_ms > interval_end_period_in_ms ||
      interval_end_period_in_ms < 0) {
    return;
  }
  double value = interval_end_period_in_ms - interval_start_period_in_ms;
  if (!average_) {
    average_ = window_average_factory_.CreateWindowedAverage(value);
  } else {
    average_->AddSample(value);
  }
}

PercentageOfIntervalMetric::PercentageOfIntervalMetric(
    MonitorSummary& monitor_summary, Monitor& monitor, absl::string_view prefix,
    absl::string_view numerator_start, absl::string_view numerator_end,
    absl::string_view denominator)
    : window_average_factory_(monitor_summary.windowed_average_factory_),
      prefix_(prefix),
      numerator_start_data_(GetOrAddDurationMeasurementData(
          monitor_summary, monitor, numerator_start)),
      numerator_end_data_(GetOrAddDurationMeasurementData(
          monitor_summary, monitor, numerator_end)),
      denominator_data_(GetOrAddDurationMeasurementData(
          monitor_summary, monitor, denominator)) {}
void PercentageOfIntervalMetric::Update() {
  if (numerator_start_data_->GetSampleCount() < 1 ||
      numerator_end_data_->GetSampleCount() < 1 ||
      denominator_data_->GetSampleCount() < 1) {
    return;
  }
  double start_period_in_ms =
      absl::ToDoubleMilliseconds(numerator_start_data_->GetTotalDuration()) /
      numerator_start_data_->GetSampleCount();
  double end_period_in_ms =
      absl::ToDoubleMilliseconds(numerator_end_data_->GetTotalDuration()) /
      numerator_end_data_->GetSampleCount();
  double denominator_period_in_ms =
      absl::ToDoubleMilliseconds(denominator_data_->GetTotalDuration()) /
      denominator_data_->GetSampleCount();

  if (start_period_in_ms > end_period_in_ms || end_period_in_ms < 0 ||
      denominator_period_in_ms <= 0) {
    return;
  }
  double percentage_of_interval = 100.0 *
                                  (end_period_in_ms - start_period_in_ms) /
                                  denominator_period_in_ms;
  if (!average_) {
    average_ =
        window_average_factory_.CreateWindowedAverage(percentage_of_interval);
  } else {
    average_->AddSample(percentage_of_interval);
  }
}
std::string PercentageOfIntervalMetric::ToString() const {
  if (!average_) {
    return std::string{};
  }
  return absl::StrCat(prefix_, *average_);
}

PercentageOfDurationMetric::PercentageOfDurationMetric(
    MonitorSummary& monitor_summary, Monitor& monitor, absl::string_view prefix,
    absl::string_view numerator, absl::string_view denominator)
    : window_average_factory_(monitor_summary.windowed_average_factory_),
      prefix_(prefix),
      numerator_data_(
          GetOrAddDurationMeasurementData(monitor_summary, monitor, numerator)),
      denominator_data_(GetOrAddDurationMeasurementData(
          monitor_summary, monitor, denominator)) {}
std::string PercentageOfDurationMetric::ToString() const {
  if (!average_) {
    return std::string{};
  }
  return absl::StrCat(prefix_, *average_);
}
void PercentageOfDurationMetric::Update() {
  if (numerator_data_->GetTotalDuration() < absl::ZeroDuration() ||
      denominator_data_->GetTotalDuration() <= absl::ZeroDuration()) {
    return;
  }
  double value = 100.0 * FDivDuration(numerator_data_->GetTotalDuration(),
                                      denominator_data_->GetTotalDuration());
  if (!average_) {
    average_ = window_average_factory_.CreateWindowedAverage(value);
  } else {
    average_->AddSample(value);
  }
}

PercentageOfCountMetric::PercentageOfCountMetric(
    MonitorSummary& monitor_summary, Monitor& monitor, absl::string_view prefix,
    absl::string_view numerator, absl::string_view denominator)
    : window_average_factory_(monitor_summary.windowed_average_factory_),
      prefix_(prefix),
      numerator_data_(
          GetOrAddDurationMeasurementData(monitor_summary, monitor, numerator)),
      denominator_data_(GetOrAddDurationMeasurementData(
          monitor_summary, monitor, denominator)) {}

std::string PercentageOfCountMetric::ToString() const {
  if (!average_) {
    return std::string{};
  }
  return absl::StrCat(prefix_, *average_);
}
void PercentageOfCountMetric::Update() {
  if (numerator_data_->GetSampleCount() < 0 ||
      denominator_data_->GetSampleCount() < 1) {
    return;
  }
  double value = 100.0 * numerator_data_->GetSampleCount() /
                 denominator_data_->GetSampleCount();
  if (!average_) {
    average_ = window_average_factory_.CreateWindowedAverage(value);
  } else {
    average_->AddSample(value);
  }
}

PercentageOfHistogramMetric::PercentageOfHistogramMetric(
    MonitorSummary& monitor_summary, Monitor& monitor, absl::string_view prefix,
    absl::string_view metric_with_histogram,
    std::optional<uint64_t> lower_bound_ms,
    std::optional<uint64_t> upper_bound_ms)
    : window_average_factory_(monitor_summary.windowed_average_factory_),
      prefix_(prefix),
      metric_with_histogram_data_(GetOrAddDurationMeasurementData(
          monitor_summary, monitor, metric_with_histogram)),
      lower_bound_ms_(lower_bound_ms),
      upper_bound_ms_(upper_bound_ms) {}

PercentageOfHistogramMetric::PercentageOfHistogramMetric(
    MonitorSummary& monitor_summary, Monitor& monitor, absl::string_view prefix,
    absl::string_view metric_with_histogram)
    : window_average_factory_(monitor_summary.windowed_average_factory_),
      prefix_(prefix),
      metric_with_histogram_data_(GetOrAddDurationMeasurementData(
          monitor_summary, monitor, metric_with_histogram)),
      lower_bound_ms_(std::nullopt) {}

std::string PercentageOfHistogramMetric::ToString() const {
  if (!average_) {
    return std::string{};
  }
  std::string result = absl::StrCat(prefix_, *average_);
  if (lower_bound_ms_.has_value()) {
    absl::StrAppend(&result, " lower_bound (ms)=", lower_bound_ms_.value());
  }
  return result;
}

void PercentageOfHistogramMetric::Update() {
  if (metric_with_histogram_data_->GetSampleCount() < 1 ||
      !metric_with_histogram_data_->GetHistogram().has_value()) {
    return;
  }
  double count = metric_with_histogram_data_->GetHistogram()->CountInRange(
      lower_bound_ms_, upper_bound_ms_);
  double percentage =
      100.0 * count / metric_with_histogram_data_->GetSampleCount();
  if (!average_) {
    average_ = window_average_factory_.CreateWindowedAverage(percentage);
  } else {
    average_->AddSample(percentage);
  }
}

ValueMetric::ValueMetric(MonitorSummary& monitor_summary, Monitor& monitor,
                         absl::string_view prefix,
                         absl::string_view metric_with_value)
    : window_average_factory_(monitor_summary.windowed_average_factory_),
      prefix_(prefix) {
  MeasurementData::MeasurementId id =
      monitor.GetMeasurementId(metric_with_value);
  if (!id) {
    id = monitor.AddMeasurement(
        std::make_unique<ValueMeasurementData>(metric_with_value));
  }

  auto value_measurement_data =
      dynamic_cast<ValueMeasurementData*>(monitor.GetMeasurementData(id));
  value_measurement_data_ = value_measurement_data;
}

void ValueMetric::Update() {
  double value;

  // extract current value as a double.
  absl::variant<absl::monostate, int64_t, double> value_holder =
      value_measurement_data_->GetValue();
  if (absl::holds_alternative<int64_t>(value_holder)) {
    value = absl::get<int64_t>(value_holder);
  } else if (absl::holds_alternative<double>(value_holder)) {
    value = absl::get<double>(value_holder);
  } else {
    // no value, do not update.
    return;
  }
  if (!average_) {
    average_ = window_average_factory_.CreateWindowedAverage(value);
  }
  average_->AddSample(value);
}

std::string ValueMetric::ToString() const {
  if (!average_ || !average_->IsChanging()) {
    return std::string{};
  }
  return absl::StrCat(prefix_, *average_);
}

SampleAgeMetric::SampleAgeMetric(MonitorSummary& monitor_summary,
                                 Monitor& monitor, absl::string_view prefix)
    : window_average_factory_(monitor_summary.windowed_average_factory_),
      prefix_(prefix),
      monitor_(monitor) {}

void SampleAgeMetric::Update() {
  auto t = monitor_.GetClock()->TimeNow();
  if (!average_) {
    average_ =
        window_average_factory_.CreateWindowedAverage(absl::ToUnixMillis(t));
  } else {
    average_->AddSample(absl::ToUnixMillis(t));
  }
}

std::string SampleAgeMetric::ToString() const {
  std::optional<float3> timestamp_millis_opt = GetAverages();
  if (!timestamp_millis_opt) {
    // Avoid clutter in the summary by returning empty string when empty.
    return std::string{};
  }
  auto timestamp_millis = timestamp_millis_opt.value();
  auto time_now = monitor_.GetClock()->TimeNow();
  int64_t elapsed_seconds1 = absl::ToInt64Seconds(
      time_now - absl::FromUnixMillis(timestamp_millis[0]));
  int64_t elapsed_seconds2 = absl::ToInt64Seconds(
      time_now - absl::FromUnixMillis(timestamp_millis[1]));
  int64_t elapsed_seconds3 = absl::ToInt64Seconds(
      time_now - absl::FromUnixMillis(timestamp_millis[2]));

  return absl::StrCat(prefix_, " ", elapsed_seconds1, " / ", elapsed_seconds2,
                      " / ", elapsed_seconds3, " (seconds)");
}

}  // namespace imp
