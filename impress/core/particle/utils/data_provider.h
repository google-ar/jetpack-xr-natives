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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_DATA_PROVIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_DATA_PROVIDER_H_

#include <cstdint>

#include "core/math/quat.h"
#include "core/math/vec.h"

namespace imp::imp_particle {

// Interface used by the Particle System to store data for each particle.
class DataProvider {
 public:
  DataProvider() = default;
  virtual ~DataProvider() = default;

  // Returns the number of floats managed by the data provider.
  virtual int32_t GetNumFloats() const = 0;

  // Single float accessors.
  virtual float GetFloat(int32_t index) const = 0;
  virtual void SetFloat(int32_t index, float value) = 0;

  // Vector2f accessors.
  float2 GetVector2f(int32_t index) const;
  void SetVector2f(int32_t index, const float2& value);

  // Vector3f accessors.
  float3 GetVector3f(int32_t index) const;
  void SetVector3f(int32_t index, const float3& value);

  // Quaternion accessors.
  quatf GetQuatf(int32_t index) const;
  void SetQuatf(int32_t index, const quatf& value);
};

}  // namespace imp::imp_particle

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_DATA_PROVIDER_H_
