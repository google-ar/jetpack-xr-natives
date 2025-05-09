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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_SCOPED_DURATION_MEASUREMENT_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_SCOPED_DURATION_MEASUREMENT_H_

#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/monitor.h"

namespace imp {

// Scope based measurement of time.  The time from creation to destruction will
// be recorded.
class ScopedDurationMeasurement {
 public:
  // Time measurement begins during construction.
  // Construction does not take ownership of the monitor, but uses it.
  // The Monitor must be valid for the lifetime of this class.
  ScopedDurationMeasurement(Monitor* monitor, absl::string_view name);

  // Move only semantics.
  ScopedDurationMeasurement(ScopedDurationMeasurement&& other) = default;
  ScopedDurationMeasurement& operator=(ScopedDurationMeasurement&& other) =
      default;

  ScopedDurationMeasurement(const ScopedDurationMeasurement& other) = delete;
  ScopedDurationMeasurement& operator=(const ScopedDurationMeasurement& other) =
      delete;

  // Stops time measurement.
  ~ScopedDurationMeasurement();

  // Configures a histogram for a Scoped Duration Measurement.
  static void AddHistogram(Monitor& monitor, absl::string_view name,
                           absl::Duration lower_bound,
                           absl::Duration bucket_width, int bucket_count);

  // Ends time counting and increments cancelled sample count without
  // accumulating results.
  void CancelSample();

 private:
  Monitor* monitor_;
  MeasurementData::MeasurementId id_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_SCOPED_DURATION_MEASUREMENT_H_
