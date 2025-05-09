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

#include "core/monitor/duration_measurement_data.h"

#include <stdint.h>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/common/platform_helpers.h"
#include "core/math/moving_average.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/monitor.h"
#include "core/monitor/monitor_helpers.h"
#include "core/monitor/simple_histogram.h"
#include "mediapipe/framework/deps/clock.h"

namespace imp {

DurationMeasurementData::DurationMeasurementData(Monitor* monitor,
                                                 absl::string_view name)
    : monitor_(monitor),
      name_(name),
      in_progress_(false),
      start_time_(absl::InfinitePast()),
      end_time_(absl::InfinitePast()),
      sample_count_(0),
      average_sample_duration_ms_(0),
      cancelled_sample_count_(0) {}

DurationMeasurementData::~DurationMeasurementData() {}

absl::string_view DurationMeasurementData::GetName() const { return name_; }

void DurationMeasurementData::AddHistogram(Monitor& monitor,
                                           absl::string_view name,
                                           absl::Duration lower_bound,
                                           absl::Duration bucket_width,
                                           int bucket_count) {
  MeasurementData::MeasurementId id = monitor.GetMeasurementId(name);
  if (!id) {
    id = monitor.AddMeasurement(
        std::make_unique<DurationMeasurementData>(&monitor, name));
  }

  static_cast<DurationMeasurementData*>(monitor.GetMeasurementData(id))
      ->InternalAddHistogram(lower_bound, bucket_width, bucket_count);
}

void DurationMeasurementData::EndSample() {
  if (!IsInProgress()) {
    // No sample is in progress.
    return;
  }
  in_progress_ = false;
  end_time_ = monitor_->GetClock()->TimeNow();
  AddSample(end_time_ - start_time_);
}

void DurationMeasurementData::CancelSample() {
  in_progress_ = false;
  end_time_ = start_time_;
  cancelled_sample_count_++;
}

void DurationMeasurementData::DiscardSample() {
  in_progress_ = false;
  end_time_ = start_time_;
}

void DurationMeasurementData::AddSample(absl::Duration sample) {
  if (IsInProgress()) {
    IMP_LOG(imp::ERROR) << "Overlapping samples are not supported.";
    return;
  }
  latest_sample_ = sample;

  double sample_ms = absl::ToDoubleMilliseconds(sample);
  sample_count_++;
  if (sample_count_ == 1) {
    total_duration_ = sample;
    shortest_sample_duration_ = sample;
    longest_sample_duration_ = sample;
    average_sample_duration_ms_ = MovingAverage(sample_ms);
  } else {
    total_duration_ += sample;
    shortest_sample_duration_ = std::min(shortest_sample_duration_, sample);
    longest_sample_duration_ = std::max(longest_sample_duration_, sample);
    average_sample_duration_ms_.AddSample(sample_ms);
  }

  if (histogram_.has_value()) {
    histogram_->Add(absl::ToInt64Milliseconds(sample));
  }
}

void DurationMeasurementData::BeginSample() {
  if (IsInProgress()) {
    IMP_LOG(imp::ERROR) << "Overlapping samples are not supported.";
    return;
  }

  start_time_ = monitor_->GetClock()->TimeNow();
  in_progress_ = true;
}

bool DurationMeasurementData::IsInProgress() const { return in_progress_; }

uint64_t DurationMeasurementData::GetSampleCount() const {
  return sample_count_;
}

absl::Duration DurationMeasurementData::GetTotalDuration() const {
  return total_duration_;
}

uint64_t DurationMeasurementData::GetCancelledSampleCount() const {
  return cancelled_sample_count_;
}

void DurationMeasurementData::Reset() {
  in_progress_ = false;
  start_time_ = monitor_->GetClock()->TimeNow();
  end_time_ = start_time_;
  sample_count_ = 0;
  total_duration_ = absl::ZeroDuration();
  longest_sample_duration_ = absl::ZeroDuration();
  shortest_sample_duration_ = absl::ZeroDuration();
  average_sample_duration_ms_ = MovingAverage(0);
  cancelled_sample_count_ = 0;

  if (histogram_.has_value()) {
    histogram_->Reset();
  }
}

void DurationMeasurementData::InternalAddHistogram(absl::Duration lower_bound,
                                                   absl::Duration bucket_width,
                                                   int bucket_count) {
  if (!(bucket_width > absl::ZeroDuration()) || bucket_count < 1) {
    histogram_.reset();
    return;
  }
  histogram_.emplace(SimpleHistogram(absl::ToInt64Milliseconds(lower_bound),
                                     absl::ToInt64Milliseconds(bucket_width),
                                     bucket_count));
}

}  // namespace imp
