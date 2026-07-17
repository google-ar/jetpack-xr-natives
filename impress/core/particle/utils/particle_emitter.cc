/*
 * Copyright 2026 Google LLC
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

#include "core/particle/utils/particle_emitter.h"

#include <memory>
#include <string>
#include <utility>

#include "core/common/log.h"
#include "core/camera/camera_component.h"
#include "core/camera/camera_manager.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/particle/custom_particle_behavior.h"
#include "core/particle/particle_emitter_info.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/utils/particle_behavior.h"
#include "core/particle/utils/particle_pool.h"
#include "core/view/utils/frame_time.h"

namespace imp::imp_particle {

ParticleEmitter::ParticleEmitter(
    NodeHandle emitter_node, const ParticleEmitterState& emitter_state,
    std::unique_ptr<CustomParticleBehavior> custom_particle_behavior)
    : emitter_node_(emitter_node),
      particle_pool_(emitter_state.particle_config.Value(),
                     emitter_state.emitter_config->max_particles.Value(),
                     emitter_node),
      particle_behavior_(emitter_node, emitter_state.particle_config.Value(),
                         std::move(custom_particle_behavior)),
      emitter_duration_finite_(
          emitter_state.emitter_config->duration_in_seconds.Value() > 0.0f),
      remaining_emitter_duration_(
          emitter_state.emitter_config->duration_in_seconds.Value()),
      emitter_duration_(
          emitter_state.emitter_config->duration_in_seconds.Value()),
      looping_(emitter_state.emitter_config->loop.Value()),
      particles_per_second_(
          emitter_state.emitter_config->particles_per_second.Value()) {}

void ParticleEmitter::UpdateEmitterBehavior(const FrameTime& frame_time) {
  // Update the remaining duration.
  if (emitter_duration_finite_) {
    // Reduce the emitter lifetime.
    if (remaining_emitter_duration_ > 0.0f) {
      remaining_emitter_duration_ -= frame_time.GetDeltaSeconds();
    }

    // If the emitter has expired, but is looping, reset its duration.
    if (looping_ && remaining_emitter_duration_ <= 0.0f) {
      remaining_emitter_duration_ = emitter_duration_;
    }
  }
}

bool ParticleEmitter::CanEmitParticles() {
  // Are emissions paused?
  if (particle_emission_paused_) return false;

  // Waiting for the next time to emit a particle?
  if (particle_delay_ > 0.0f) return false;

  // Have we reached the maximum number of active particles?
  if (particle_pool_.GetNumAvailableParticles() <= 0) {
    return false;
  }

  // Is the emitter actively emitting particles?
  if (emitter_duration_finite_ && remaining_emitter_duration_ <= 0.0f)
    return false;

  return true;
}

imp::ParticleEmitterInfo ParticleEmitter::GetParticleEmitterInfo() const {
  // Get the current camera position.
  float3 camera_position = kZero3;
  if (emitter_node_.IsValid()) {
    ComponentHandle<CameraComponent> camera =
        emitter_node_->GetView().GetCameraManager().GetCamera();
    if (camera.IsValid()) {
      camera_position = camera->GetNode()->GetWorldPosition();
    }
  }

  // Assemble the emitter info.
  return imp::ParticleEmitterInfo(camera_position);
}

std::string ParticleEmitter::ValidateEmitterState(
    const ParticleEmitterState& emitter_state) {
  // The emitter config must be set.
  if (!emitter_state.emitter_config) {
    return "ParticleEmitter - emitter_config not set!";
  }

  // The particle config must be set.
  if (!emitter_state.particle_config) {
    return "ParticleEmitter - particle_config not set!";
  }

  // Particles must be emitted.
  if (emitter_state.emitter_config->particles_per_second.Value() <= 0.0f) {
    return "ParticleEmitter - particles per second must be > 0!";
  }

  // There must be an allowance for the maximum number of particles.
  if (emitter_state.emitter_config->max_particles.Value() <= 0) {
    return "ParticleEmitter - max particles must be > 0!";
  }

  // The duration must be non-negative.
  if (emitter_state.emitter_config->duration_in_seconds.Value() < 0.0f) {
    return "ParticleEmitter - duration in seconds must be >= 0!";
  }

  return "";
}

void ParticleEmitter::SetEmissionRate(float particles_per_second) {
  if (particles_per_second > 0.0f) {
    particles_per_second_ = particles_per_second;
  } else {
    IMP_LOG(imp::ERROR) << "ParticleEmitter - particles per second must be > 0!";
  }
}

}  // namespace imp::imp_particle
