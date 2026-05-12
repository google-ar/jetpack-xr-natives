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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_EMITTER_INFO_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_EMITTER_INFO_H_

#include "core/math/vec.h"

namespace imp {
namespace imp_particle {

// This object stores emitter information relevant to all particles being
// updated each frame. For example, the camera position will be used by all
// particles with the billboard behavior. This object is not intended to last
// beyond the frame in which it is created.
class ParticleEmitterInfo {
 public:
  ParticleEmitterInfo(float3 camera_position)
      : camera_position_(camera_position) {};

  // Returns the position of the camera in world space, for the current frame.
  float3 GetCameraPosition() const { return camera_position_; }

 private:
  float3 camera_position_ = kZero3;
};

}  // namespace imp_particle
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_EMITTER_INFO_H_
