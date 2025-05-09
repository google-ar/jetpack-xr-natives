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

#include "core/monitor/value_measurement_data.h"

#include <string>

#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/monitor/value_measurement.h"

namespace imp {

ValueMeasurementData::ValueMeasurementData(absl::string_view name)
    : name_(name) {}

ValueMeasurementData::ValueMeasurementData(absl::string_view name,
                                           MonitorValue value)
    : name_(name), value_(value) {}

ValueMeasurementData::~ValueMeasurementData() {}

absl::string_view ValueMeasurementData::GetName() const { return name_; }

void ValueMeasurementData::SetValue(MonitorValue value) { value_ = value; }

const ValueMeasurement::MonitorValue& ValueMeasurementData::GetValue() const {
  return value_;
}

void ValueMeasurementData::Reset() { value_ = absl::monostate(); }

}  // namespace imp
