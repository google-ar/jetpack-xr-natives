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

#include "core/monitor/value_measurement.h"

#include <memory>

#include "absl/strings/string_view.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/monitor.h"
#include "core/monitor/value_measurement_data.h"

namespace imp {

ValueMeasurement::ValueMeasurement(Monitor& monitor, absl::string_view name)
    : monitor_(monitor) {
  id_ = monitor_.GetMeasurementId(name);
  if (!id_) {
    id_ = monitor_.AddMeasurement(std::make_unique<ValueMeasurementData>(name));
  }
}

ValueMeasurement::ValueMeasurement(Monitor& monitor, absl::string_view name,
                                   MonitorValue initial_value)
    : ValueMeasurement(monitor, name) {
  SetValue(initial_value);
}

void ValueMeasurement::SetValue(MonitorValue value) {
  ValueMeasurementData* measurement =
      static_cast<ValueMeasurementData*>(monitor_.GetMeasurementData(id_));
  measurement->SetValue(value);
}

const ValueMeasurement::MonitorValue& ValueMeasurement::GetValue() const {
  ValueMeasurementData* measurement =
      static_cast<ValueMeasurementData*>(monitor_.GetMeasurementData(id_));
  return measurement->GetValue();
}

}  // namespace imp
