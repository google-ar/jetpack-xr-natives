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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_STANDARD_PARTICLE_SERVICE_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_STANDARD_PARTICLE_SERVICE_H_

#include <cstdint>

#include "core/async/future.h"
#include "core/particle/data_layout.h"
#include "core/particle/particle_data.h"
#include "core/particle/particle_data_provider.h"
#include "core/particle/particle_emitter_info.h"
#include "core/particle/particle_service.h"
#include "core/particle/standard_particle_data_provider.h"
#include "core/view/utils/frame_time.h"

namespace imp {

using imp_particle::DataLayout;

// The Standard Particle Service is used by Particle Emitters to manage the
// storage of particle state data and their lifecycle management. It is intended
// to provide an implementation that supports the most common types of particle
// systems. Users can extend or replace this service with a custom
// implementation if required.
class StandardParticleService : public ParticleService {
 public:
  ~StandardParticleService();

  // Creates a new StandardParticleService. This implementation of a
  // ParticleService does not require async initialization, the future will be
  // ready immediately upon return.
  static Future<OwnedParticleServicePtr> Create(/* particle_description */);

  // Performs common maintenance tasks for all active particles.
  void ProcessActiveParticles(const FrameTime& frame_time) override;

 protected:
  // Creates a ParticleData instance for a specific particle.
  ParticleData GetDataForParticle(int32_t particle_index) override;

  // Creates ParticleEmitterInfo for this service.
  ParticleEmitterInfo GetEmitterInfo() const override;

  // Access to the underlying data provider that holds the particle state data.
  ParticleDataProvider& GetDataProvider() override;

 private:
  StandardParticleService(/* particle_description */);

  DataLayout data_layout_;
  StandardParticleDataProvider data_provider_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_STANDARD_PARTICLE_SERVICE_H_
