/*
 * Copyright 2025 Google LLC
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

#include "core/particle/utils/standard_data_provider.h"

#include <cstdint>
#include <vector>

#include "core/common/log.h"

namespace imp::imp_particle {

StandardDataProvider::StandardDataProvider(int32_t num_floats) {
  if (num_floats < 0) {
    IMP_LOG(imp::FATAL) << "StandardDataProvider, invalid number of floats!";
  }

  float_data_.resize(num_floats);
}

int32_t StandardDataProvider::GetNumFloats() const {
  return float_data_.size();
}

float StandardDataProvider::GetFloat(int32_t index) const {
  return float_data_[index];
}

void StandardDataProvider::SetFloat(int32_t index, float value) {
  float_data_[index] = value;
}

}  // namespace imp::imp_particle
