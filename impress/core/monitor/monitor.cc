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

#include "core/monitor/monitor.h"

#include <iterator>
#include <memory>
#include <string>
#include <utility>

#include "absl/strings/string_view.h"
#include "mediapipe/framework/deps/clock.h"
#include "absl/time/time.h"
#include "core/common/typed_container_helpers.h"
#include "core/common/typed_id.h"
#include "core/common/typed_vector.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/monitor_helpers.h"
#include "core/monitor/profiling_clock.h"
#include "core/view/utils/string_map.h"
#include "robin_map/include/tsl/robin_hash.h"

namespace imp {

Monitor::Monitor()
    : clock_(std::make_unique<ProfilingClock>()),
      start_time_(clock_->TimeNow()) {}

void Monitor::Reset() {
  for (auto iter = measurements_.begin(); iter != measurements_.end(); ++iter) {
    auto measurement_data = (*iter).get();
    if (measurement_data != nullptr) {
      measurement_data->Reset();
    }
  }
  start_time_ = GetClock()->TimeNow();
}

MeasurementData* Monitor::GetMeasurementData(
    MeasurementData::MeasurementId id) const {
  if (!measurements_.IsValid(id)) {
    return nullptr;
  }
  return measurements_[id].get();
}

MeasurementData::MeasurementId Monitor::GetMeasurementId(
    absl::string_view name) const {
  auto iter = measurement_name_vs_id_.find(name);
  if (iter == measurement_name_vs_id_.end()) {
    return MeasurementData::MeasurementId();
  }
  return iter.value();
}

MeasurementData::MeasurementId Monitor::AddMeasurement(
    std::unique_ptr<MeasurementData> new_measurement) {
  std::string name(new_measurement->GetName());
  auto id = measurements_.Append<MeasurementData::MeasurementId>(
      std::move(new_measurement));
  measurement_name_vs_id_[name] = id;
  return id;
}

void Monitor::SetClock(std::unique_ptr<mediapipe::Clock> clock) {
  clock_ = std::move(clock);
}

}  // namespace imp
