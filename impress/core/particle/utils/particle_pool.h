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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_POOL_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_POOL_H_

#include <cstdint>
#include <list>

#include "core/ncsb/node_handle.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/particle_instance.h"
#include "core/particle/utils/particle_format.h"
#include "core/particle/utils/standard_data_provider.h"

namespace imp::imp_particle {

// The ParticlePool manages the storage for each particle in the system. It
// provides facilities to create and destroy particles as needed, and provides a
// ParticleInstance accessor for an active particle, that allows users to
// inspect and modify the state of that particle.
class ParticlePool {
 public:
  // Initializes the pool.
  ParticlePool(const ParticleConfig& particle_config, int32_t max_particles,
               NodeHandle emitter_node);

  // Returns a ParticleInstance for the given particle_index. The object can
  // be used to inspect and modify the state of the particle for any behavior
  // it was configured to perform. It should not be held beyond the current
  // frame.
  ParticleInstance GetParticleInstance(int32_t particle_index);

  // Allocates a particle from the pool of available particles. The state of the
  // particle is undefined when returned and should be initialized by the
  // caller. `kInvalidParticleIndex`, if no particles are available.
  int32_t CreateParticle();

  // Releases the particle resources back to the common pool. The instance must
  // not be held after this call.
  void DestroyParticle(int32_t particle_index);

  // Returns the maximum number of particles that may be active.
  int32_t GetMaxParticles() const { return max_particles_; }

  // Returns the number of particles that are available to be used.
  int32_t GetNumAvailableParticles() const {
    return free_particle_indices_.size();
  }

 private:
  ParticleFormat particle_format_;
  StandardDataProvider data_provider_;
  int32_t max_particles_;
  std::list<int32_t> free_particle_indices_;
  NodeHandle emitter_node_;
};

}  // namespace imp::imp_particle

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_POOL_H_
