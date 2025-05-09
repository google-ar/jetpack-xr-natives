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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_VALUE_MEASUREMENT_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_VALUE_MEASUREMENT_DATA_H_

#include <stdint.h>

#include <string>
#include <variant>

#include "absl/strings/string_view.h"
#include "core/monitor/measurement_data.h"
#include "core/monitor/value_measurement.h"

namespace imp {

// The data stored for ValueMeasurements
class ValueMeasurementData : public MeasurementData {
 public:
  using MonitorValue = absl::variant<absl::monostate, int64_t, double>;

  explicit ValueMeasurementData(absl::string_view name);
  ValueMeasurementData(absl::string_view name, MonitorValue value);
  ~ValueMeasurementData() override;

  void SetValue(MonitorValue value);
  const MonitorValue& GetValue() const;

  absl::string_view GetName() const override;
  void Reset() override;

 private:
  std::string name_;
  MonitorValue value_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_VALUE_MEASUREMENT_DATA_H_
