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

#include "core/particle/particle_system.h"

#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/particle/custom_particle_behavior.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/utils/node_particle_emitter.h"
#include "core/particle/utils/particle_emitter.h"
#include "core/view/utils/frame_time.h"

namespace imp {

Future<absl::Status> ParticleSystem::SetupWithState(
    std::unique_ptr<CustomParticleBehavior> custom_particle_behavior) {
  // Create the particle emitter based on the defined render type.
  if (state_.renderer == ParticleEmitterState::PARTICLE_RENDERER_NODE) {
    return imp_particle::NodeParticleEmitter::Create(
               GetNode(), state_, std::move(custom_particle_behavior))
        .Then([this](imp_particle::OwnedParticleEmitterPtr emitter) {
          emitter_ = std::move(emitter);
        });
  }

  // No other render types are supported yet.
  return Future<absl::Status>(
      absl::UnimplementedError("Unsupported particle renderer type."));
}

void ParticleSystem::Update(const FrameTime& frame_time) {
  emitter_->UpdateParticleSystem(frame_time);
}

float ParticleSystem::GetEmissionRate() const {
  return emitter_->GetEmissionRate();
}

void ParticleSystem::SetEmissionRate(float particles_per_second) {
  emitter_->SetEmissionRate(particles_per_second);
}

bool ParticleSystem::IsEmissionPaused() const {
  return emitter_->IsEmissionPaused();
}

void ParticleSystem::SetEmissionPaused(bool pause) {
  emitter_->SetEmissionPaused(pause);
}

}  // namespace imp
