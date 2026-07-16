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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_FORMAT_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_FORMAT_H_

#include <cstdint>
#include <vector>

#include "core/particle/particle_emitter_state.proto.imp.h"

namespace imp::imp_particle {

const int32_t kInvalidParticleDataOffset = -1;

// Used by the Particle System to define the attributes used by particles in
// the system. This is analogous to a Vertex Format. It can be configured to
// include any subset of the available fields, and provides callers with
// accessors for the offset to each field value within an array of values.
class ParticleFormat {
 public:
  ParticleFormat(const ParticleConfig& config);

  // Returns the number of float values needed based on the Setup description.
  int GetSize() const;

  // Returns the offset of the named field from the start of the particle data.
  // If the field is not used, kInvalidParticleDataOffset is returned.
  int GetLifetime() const;
  int GetAlpha() const;
  int GetScale() const;
  int GetPosition() const;
  int GetVelocity() const;
  int GetAcceleration() const;
  int GetRotation() const;

 private:
  std::vector<int> offsets_;
  int particle_data_size_ = 0;
};

}  // namespace imp::imp_particle

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_FORMAT_H_
