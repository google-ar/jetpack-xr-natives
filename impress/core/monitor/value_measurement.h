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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_VALUE_MEASUREMENT_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_VALUE_MEASUREMENT_H_

#include <stdint.h>

#include <variant>

#include "absl/strings/string_view.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/monitor.h"

namespace imp {

// The simplest form of measurement, this can be set to any number and will
// be reported directly.
class ValueMeasurement {
 public:
  // TODO: make a logging proto to write out MonitorValue. Add
  // Report function to write proto.
  using MonitorValue = absl::variant<absl::monostate, int64_t, double>;

  // Construction does not take ownership of the monitor, but uses it.
  // The Monitor must be valid for the lifetime of this class.
  ValueMeasurement(Monitor& monitor, absl::string_view name);
  ValueMeasurement(Monitor& monitor, absl::string_view name,
                   MonitorValue value);
  void SetValue(MonitorValue value);
  const MonitorValue& GetValue() const;

 private:
  Monitor& monitor_;
  MeasurementData::MeasurementId id_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_VALUE_MEASUREMENT_H_
