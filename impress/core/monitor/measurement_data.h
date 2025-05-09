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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_MEASUREMENT_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_MEASUREMENT_DATA_H_
#include <stdint.h>

#include <memory>

#include "absl/strings/string_view.h"
#include "core/common/typed_id.h"

namespace imp {

/**
 * Datastore for measurement data.
 *
 * The Monitor holds MeasurementData and can communicate with it directly.
 * MeasurementData may accumulate multiple results to make a report.
 */
class MeasurementData {
 public:
  using MeasurementId =
      ::imp::TypedId<std::unique_ptr<MeasurementData>, int32_t>;

  MeasurementData() = default;
  virtual ~MeasurementData() = default;

  // Returns the name of the measurement.  The name must be unique for a
  // measurement.
  virtual absl::string_view GetName() const = 0;

  // Clears data store.
  virtual void Reset() = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_MEASUREMENT_DATA_H_
