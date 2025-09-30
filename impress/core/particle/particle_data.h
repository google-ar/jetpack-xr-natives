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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_DATA_H_

#include <cstdint>

#include "core/math/vec.h"
#include "core/particle/data_layout.h"
#include "core/particle/particle_data_provider.h"

namespace imp {

using imp_particle::DataLayout;

// An object that facilitates access to the current state of a particle.
// Users of custom callbacks will be provided with an instance of this class
// for each particle the callback is invoked for. The class is also used
// internally by the particle system.
class ParticleData {
 public:
  ParticleData(ParticleDataProvider& data_provider,
               const DataLayout& data_layout, int32_t float_offset);

  // Returns the time remaining for a particle to remain active.
  float GetRemainingLifetimeSeconds() const;

  // Returns the current alpha value for a particle.
  float GetAlpha() const;
  void SetAlpha(float alpha);

  // Returns the current scale value for a particle.
  float GetScale() const;
  void SetScale(float scale);

  // Returns the current position value for a particle.
  float3 GetPosition() const;
  void SetPosition(float3 position);

  // Returns the current velocity value for a particle.
  float3 GetVelocity() const;
  void SetVelocity(float3 velocity);

 private:
  ParticleDataProvider& data_provider_;
  const DataLayout& data_layout_;
  int32_t float_offset_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_DATA_H_
