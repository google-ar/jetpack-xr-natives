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

#include "core/particle/utils/data_provider.h"

#include <cstdint>

#include "core/math/quat.h"
#include "core/math/vec.h"

namespace imp::imp_particle {

float2 DataProvider::GetVector2f(int32_t index) const {
  return float2(GetFloat(index), GetFloat(index + 1));
}

void DataProvider::SetVector2f(int32_t index, const float2& value) {
  SetFloat(index, value.x);
  SetFloat(index + 1, value.y);
}

float3 DataProvider::GetVector3f(int32_t index) const {
  return float3(GetFloat(index), GetFloat(index + 1), GetFloat(index + 2));
}

void DataProvider::SetVector3f(int32_t index, const float3& value) {
  SetFloat(index, value.x);
  SetFloat(index + 1, value.y);
  SetFloat(index + 2, value.z);
}

quatf DataProvider::GetQuatf(int32_t index) const {
  // Restoring from (w, x, y, z) to match the quaternion constructor.
  return quatf(GetFloat(index), GetFloat(index + 1), GetFloat(index + 2),
               GetFloat(index + 3));
}

void DataProvider::SetQuatf(int32_t index, const quatf& value) {
  // Storing to (w, x, y, z) to match the quaternion constructor.
  SetFloat(index, value.w);
  SetFloat(index + 1, value.x);
  SetFloat(index + 2, value.y);
  SetFloat(index + 3, value.z);
}

}  // namespace imp::imp_particle
