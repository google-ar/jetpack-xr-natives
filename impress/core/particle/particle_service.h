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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_SERVICE_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_SERVICE_H_

#include <cstdint>

#include "core/common/owned_ptr.h"
#include "core/particle/particle_data.h"
#include "core/particle/particle_data_provider.h"
#include "core/particle/particle_emitter_info.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// The Particle Service represents the API interface for services that manage
// the storage of particle state data and their lifecycle management. This
// interface must be implemented in a derived class to be used by the
// Particle Emitter.
class ParticleService {
 public:
  ParticleService() = default;
  virtual ~ParticleService() = default;

  // Performs common maintenance tasks for all active particles.
  virtual void ProcessActiveParticles(const FrameTime& frame_time) = 0;

 protected:
  virtual ParticleData GetDataForParticle(int32_t particle_index) = 0;
  virtual ParticleEmitterInfo GetEmitterInfo() const = 0;
  virtual ParticleDataProvider& GetDataProvider() = 0;
};

// OwnedPtr definition for ParticleServices.
using OwnedParticleServicePtr = OwnedPtr<ParticleService>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_SERVICE_H_
