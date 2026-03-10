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

#include "core/particle/particle_emitter.h"

#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/particle/custom_particle_behavior.h"
#include "core/particle/node_particle_controller.h"
#include "core/particle/particle_controller.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/view/utils/frame_time.h"

namespace imp {

Future<absl::Status> ParticleEmitter::SetupWithState(
    std::unique_ptr<CustomParticleBehavior> custom_particle_behavior) {
  // Create the particle controller based on the defined render type.
  if (state_.renderer == ParticleEmitterState::PARTICLE_RENDERER_NODE) {
    return NodeParticleController::Create(GetNode(), state_,
                                          std::move(custom_particle_behavior))
        .Then([this](OwnedParticleControllerPtr controller) {
          controller_ = std::move(controller);
        });
  }

  // No other render types are supported yet.
  return Future<absl::Status>(
      absl::UnimplementedError("Unsupported particle renderer type."));
}

void ParticleEmitter::Update(const FrameTime& frame_time) {
  controller_->UpdateParticleSystem(frame_time);
}

}  // namespace imp
