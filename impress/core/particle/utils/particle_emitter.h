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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_EMITTER_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_EMITTER_H_

#include "core/common/owned_ptr.h"
#include "core/particle/particle_emitter_info.h"
#include "core/view/utils/frame_time.h"

namespace imp::imp_particle {

// Interface for particle emitters. A particle emitter is responsible for
// for creating and managing particles within a particle system for the
// lifetime of the system. Impress supports node-based and instanced-based
// particles.
class ParticleEmitter {
 public:
  virtual ~ParticleEmitter() = default;

  // Updates all active particles in the system, creates and destroys particles
  // as defined by the ParticleEmitterConfig.
  virtual void UpdateParticleSystem(const FrameTime& frame_time) = 0;

  // Returns information about the emitter used to update particle behavior.
  virtual imp::ParticleEmitterInfo GetParticleEmitterInfo() const = 0;
};

// OwnedPtr and BorrowedPtr definitions for ParticleEmitter.
using OwnedParticleEmitterPtr = OwnedPtr<ParticleEmitter>;
using BorrowedParticleEmitterPtr = BorrowedPtr<ParticleEmitter>;

}  // namespace imp::imp_particle

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_EMITTER_H_
