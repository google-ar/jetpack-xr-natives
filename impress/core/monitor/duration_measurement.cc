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

#include "core/monitor/duration_measurement.h"

#include <memory>

#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/monitor/duration_measurement_data.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/monitor.h"

namespace imp {

DurationMeasurement::DurationMeasurement(Monitor* monitor,
                                         absl::string_view name)
    : monitor_(monitor) {
  id_ = monitor_->GetMeasurementId(name);
  if (!id_) {
    id_ = monitor_->AddMeasurement(
        std::make_unique<DurationMeasurementData>(monitor, name));
  }
}

void DurationMeasurement::AddHistogram(Monitor& monitor, absl::string_view name,
                                       absl::Duration lower_bound,
                                       absl::Duration bucket_width,
                                       int bucket_count) {
  DurationMeasurementData::AddHistogram(monitor, name, lower_bound,
                                        bucket_width, bucket_count);
}

void DurationMeasurement::SetEnableAverages(bool enable_averages) {
  static_cast<DurationMeasurementData*>(monitor_->GetMeasurementData(id_))
      ->SetEnableAverages(enable_averages);
}

void DurationMeasurement::SetEnablePercentiles(bool enable_percentiles) {
  static_cast<DurationMeasurementData*>(monitor_->GetMeasurementData(id_))
      ->SetEnablePercentiles(enable_percentiles);
}

void DurationMeasurement::BeginSample() {
  static_cast<DurationMeasurementData*>(monitor_->GetMeasurementData(id_))
      ->BeginSample();
}

void DurationMeasurement::EndSample() {
  static_cast<DurationMeasurementData*>(monitor_->GetMeasurementData(id_))
      ->EndSample();
}

void DurationMeasurement::CancelSample() {
  static_cast<DurationMeasurementData*>(monitor_->GetMeasurementData(id_))
      ->CancelSample();
}

void DurationMeasurement::DiscardSample() {
  static_cast<DurationMeasurementData*>(monitor_->GetMeasurementData(id_))
      ->DiscardSample();
}

void DurationMeasurement::AddSample(absl::Duration sample) {
  static_cast<DurationMeasurementData*>(monitor_->GetMeasurementData(id_))
      ->AddSample(sample);
}

bool DurationMeasurement::IsInProgress() const {
  return static_cast<DurationMeasurementData*>(
             monitor_->GetMeasurementData(id_))
      ->IsInProgress();
}

}  // namespace imp
