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
#include <list>

#include "core/particle/data_layout.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/particle_instance.h"
#include "core/particle/standard_particle_data_provider.h"

namespace imp {

// The Particle Service object manages the current state of each particle in
// an instance of a particle system. It provides facilities to create and
// destroy particles as needed, and provides a ParticleInstance accessor to
// allow users to inspect and modify the state of a particle.
//
// The ParticleService is owned by a Particle Controller, which implements
// the visual representation and behavior of each particle in the scene.
// For example, node based particles will pair each ParticleService particle
// instance with a scene Node, and keep them in sync each frame.
class ParticleService {
 public:
  // Initializes the service.
  ParticleService(const ParticleConfig& particle_config, int32_t max_particles);

  // Returns a ParticleInstance for the given particle_index. The object can
  // be used to inspect and modify the state of the particle for any behavior
  // it was configured to perform. It should not be held beyond the current
  // frame.
  ParticleInstance GetParticleInstance(int32_t particle_index);

  // Creates a new particle from the pool of available particles.
  int32_t CreateParticle();

  // Destroys a particle, returning its resources to the pool of available
  // particles.
  void DestroyParticle(int32_t particle_index);

 private:
  imp_particle::DataLayout data_layout_;
  StandardParticleDataProvider data_provider_;
  std::list<int32_t> free_particle_indices_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_SERVICE_H_
