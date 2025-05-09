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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_MONITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_MONITOR_H_

#include <memory>

#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/common/typed_vector.h"
#include "core/monitor/measurement_data.h"
#include "core/view/utils/string_map.h"
#include "mediapipe/framework/deps/clock.h"

namespace imp {
/*
 * Interface for continuous performance monitoring.
 */
class Monitor {
 public:
  Monitor();

  // Clears accumulated Measurements.
  void Reset();

  // Returns the data store matching this id if there is one.
  MeasurementData* GetMeasurementData(MeasurementData::MeasurementId id) const;

  // Returns the data store matching this name if there is one.
  // TODO: enforce type safety in Monitor::GetMeasurementId
  MeasurementData::MeasurementId GetMeasurementId(absl::string_view name) const;

  // Creates a new measurement and provides an id for it.
  MeasurementData::MeasurementId AddMeasurement(
      std::unique_ptr<MeasurementData> new_measurement);

  absl::Time GetStartTime() const { return start_time_; }

  mediapipe::Clock* GetClock() const { return clock_.get(); }

  // Replaces the clock to be used for measurement.
  void SetClock(std::unique_ptr<mediapipe::Clock> clock);

 private:
  StringMap<MeasurementData::MeasurementId> measurement_name_vs_id_;
  TypedVector<std::unique_ptr<MeasurementData>> measurements_;
  std::unique_ptr<mediapipe::Clock> clock_;
  absl::Time start_time_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_MONITOR_H_
