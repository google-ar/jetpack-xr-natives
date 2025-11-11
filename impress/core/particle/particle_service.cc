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

#include "core/particle/particle_service.h"

#include <cstdint>

#include "core/particle/data_layout.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/particle_instance.h"
#include "core/particle/standard_particle_data_provider.h"

namespace imp {

ParticleService::ParticleService(const ParticleConfig& particle_config,
                                 int32_t max_particles)
    : data_layout_(particle_config),
      data_provider_(max_particles * data_layout_.GetSize()) {
  // Initialize the list of free particle indices.
  for (int i = 0; i < max_particles; i++) {
    free_particle_indices_.push_back(i);
  }
}

ParticleInstance ParticleService::GetParticleInstance(int32_t particle_index) {
  return ParticleInstance(data_provider_, data_layout_, particle_index);
}

int32_t ParticleService::CreateParticle() {
  // Verify there is an available particle record.
  if (free_particle_indices_.empty()) {
    return kInvalidParticleIndex;
  }

  // Allocate the next available index from the free list.
  int32_t particle_index = free_particle_indices_.front();
  free_particle_indices_.pop_front();

  // TODO: (broken link) - Initialize the instance with default values.

  return particle_index;
}

void ParticleService::DestroyParticle(int32_t particle_index) {
  // Return the particle index to the free list.
  free_particle_indices_.push_back(particle_index);
}

}  // namespace imp
