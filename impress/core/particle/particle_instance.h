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

#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/particle/particle_format.h"
#include "core/particle/utils/data_provider.h"

namespace imp {

// Invalid index value for a particle.
const int32_t kInvalidParticleIndex = -1;

// The ParticleInstance object is an accessor to the state data for a single
// particle. It is used to access and modify the current state of a particle.
class ParticleInstance {
 public:
  // TODO: (broken link) - Remove internal implementation from public API.
  ParticleInstance(imp_particle::DataProvider& data_provider,
                   const ParticleFormat& format, int32_t particle_index,
                   NodeHandle emitter_node);

  // The index of the particle represented. This index is used internally by
  // the ParticleService, and may be reused when a particle expires and its
  // resources are reused by a subsequent particle.
  int32_t GetParticleIndex() const;

  // Retrieves the emitter node that owns this particle.
  NodeHandle GetEmitterNode() const;

  // The amount of time, in seconds, that the particle will remain active.
  bool HasRemainingLifetimeSeconds() const;
  float GetRemainingLifetimeSeconds() const;
  void SetRemainingLifetimeSeconds(float seconds);

  // Alpha value for the particle's visual translucency.
  bool HasAlpha() const;
  float GetAlpha() const;
  void SetAlpha(float alpha);

  // Scaling size for the particle's visual representation.
  bool HasScale() const;
  float3 GetScale() const;
  void SetScale(float3 scale);

  // Current position, in world-space, of the particle.
  // Note: particles always have a position.
  float3 GetPosition() const;
  void SetPosition(float3 position);

  // Current particle velocity in world-space.
  bool HasVelocity() const;
  float3 GetVelocity() const;
  void SetVelocity(float3 velocity);

  // Acceleration applied to the particle's velocity over time.
  bool HasAcceleration() const;
  float3 GetAcceleration() const;
  void SetAcceleration(float3 acceleration);

  // Rotation of the particle.
  bool HasRotation() const;
  quatf GetRotation() const;
  void SetRotation(quatf rotation);

 private:
  imp_particle::DataProvider& data_provider_;
  const ParticleFormat& particle_format_;
  int32_t particle_index_ = 0;
  int32_t float_offset_ = 0;
  NodeHandle emitter_node_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_DATA_H_
