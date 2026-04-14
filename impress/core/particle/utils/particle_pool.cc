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

#include "core/particle/utils/particle_pool.h"

#include <cstdint>

#include "core/ncsb/node_handle.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/particle_format.h"
#include "core/particle/particle_instance.h"
#include "core/particle/utils/standard_data_provider.h"

namespace imp::imp_particle {

ParticlePool::ParticlePool(const ParticleConfig& particle_config,
                           int32_t max_particles, NodeHandle emitter_node)
    : particle_format_(particle_config),
      data_provider_(max_particles * particle_format_.GetSize()),
      max_particles_(max_particles),
      emitter_node_(emitter_node) {
  // Initialize the list of free particle indices.
  for (int i = 0; i < max_particles; i++) {
    free_particle_indices_.push_back(i);
  }
}

ParticleInstance ParticlePool::GetParticleInstance(int32_t particle_index) {
  return ParticleInstance(data_provider_, particle_format_, particle_index,
                          emitter_node_);
}

int32_t ParticlePool::CreateParticle() {
  // Verify there is an available particle record.
  if (free_particle_indices_.empty()) {
    return kInvalidParticleIndex;
  }

  // Allocate the next available index from the free list.
  int32_t particle_index = free_particle_indices_.front();
  free_particle_indices_.pop_front();

  return particle_index;
}

void ParticlePool::DestroyParticle(int32_t particle_index) {
  // Return the particle index to the free list.
  free_particle_indices_.push_back(particle_index);
}

}  // namespace imp::imp_particle
