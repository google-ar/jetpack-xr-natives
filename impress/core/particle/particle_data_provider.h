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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_DATA_PROVIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_DATA_PROVIDER_H_

#include <cstdint>

namespace imp {

// Interface used by the Particle System to store data for each particle.
class ParticleDataProvider {
 public:
  ParticleDataProvider() = default;
  virtual ~ParticleDataProvider() = default;

  // Returns the float value at the given index.
  virtual float GetFloat(int32_t index) const = 0;

  // Sets the float value at the given index.
  virtual void SetFloat(int32_t index, float value) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_DATA_PROVIDER_H_
