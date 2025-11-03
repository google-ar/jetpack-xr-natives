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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_EMITTER_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_EMITTER_H_

#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// The ParticleEmitter component manages the lifecycle and resources needed for
// most typical particle systems. It is configured with a ParticleEmitterState
// proto.
class ParticleEmitter : public Component {
 public:
  void Update(const FrameTime& frame_time) {};

 private:
  ParticleEmitterState state_;

 public:
  using IsfInfo = IsfInfo<&ParticleEmitter::state_>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_EMITTER_H_
