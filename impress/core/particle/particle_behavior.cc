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

#include "core/particle/particle_behavior.h"

#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/particle/particle_emitter_info.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/particle_instance.h"

namespace imp {

ParticleBehavior::ParticleBehavior(NodeHandle emitter_node,
                                   const ParticleConfig& particle_config)
    : emitter_node_(emitter_node),
      default_lifetime_seconds_(
          particle_config.lifetime_in_seconds.value_or(0.0f)),
      default_velocity_(particle_config.velocity.value_or(kZero3)),
      default_acceleration_(particle_config.acceleration.value_or(kZero3)),
      default_scale_(particle_config.scale.value_or(kOne3)) {}

void ParticleBehavior::SetDefaultValues(ParticleInstance& particle_instance) {
  // Default position of a particle is the emitter node position.
  particle_instance.SetPosition(emitter_node_->GetWorldPosition());

  // Set particle lifetime.
  if (particle_instance.HasRemainingLifetimeSeconds()) {
    particle_instance.SetRemainingLifetimeSeconds(default_lifetime_seconds_);
  }

  // Set velocity and acceleration.
  if (particle_instance.HasVelocity()) {
    particle_instance.SetVelocity(default_velocity_);
  }
  if (particle_instance.HasAcceleration()) {
    particle_instance.SetAcceleration(default_acceleration_);
  }

  // Set the scale.
  if (particle_instance.HasScale()) {
    particle_instance.SetScale(default_scale_);
  }
}

ParticleBehavior::UpdateResult ParticleBehavior::UpdateParticle(
    const imp_particle::ParticleEmitterInfo& emitter_info, float delta_seconds,
    ParticleInstance& particle_instance) {
  // Lifetime is the most common reason a particle will expire, check it first.
  if (particle_instance.HasRemainingLifetimeSeconds()) {
    UpdateResult result = UpdateLifetime(delta_seconds, particle_instance);
    if (result != ParticleBehavior::UpdateResult::kActive) {
      return result;
    }
  }

  // Update movement, dependent on Velocity, Acceleration will be updated if
  // it is also defined.
  if (particle_instance.HasVelocity()) {
    UpdateMovement(delta_seconds, particle_instance);
  }

  // Update billboard orientation to face the camera.
  if (particle_instance.HasRotation()) {
    UpdateBillboard(emitter_info, particle_instance);
  }

  // TODO: (broken link) - Support alpha behavior.

  return UpdateResult::kActive;
}

ParticleBehavior::UpdateResult ParticleBehavior::UpdateLifetime(
    float delta_seconds, ParticleInstance& particle_instance) {
  // Update the particle's remaining lifetime.
  float seconds = particle_instance.GetRemainingLifetimeSeconds();
  seconds -= delta_seconds;
  particle_instance.SetRemainingLifetimeSeconds(seconds);

  // If the lifetime has run out, notify the caller that this particle may now
  // be destroyed.
  if (seconds <= 0.0f) {
    return ParticleBehavior::UpdateResult::kExpired;
  }

  return ParticleBehavior::UpdateResult::kActive;
}

void ParticleBehavior::UpdateMovement(float delta_seconds,
                                      ParticleInstance& particle_instance) {
  // Read velocity once, it may also be modified by acceleration.
  float3 velocity = particle_instance.GetVelocity();

  // Update position.
  particle_instance.SetPosition(particle_instance.GetPosition() +
                                velocity * delta_seconds);

  // If acceleration is also defined, update velocity.
  if (particle_instance.HasAcceleration()) {
    particle_instance.SetVelocity(
        velocity + particle_instance.GetAcceleration() * delta_seconds);
  }
}

void ParticleBehavior::UpdateBillboard(
    const imp_particle::ParticleEmitterInfo& emitter_info,
    ParticleInstance& particle_instance) {
  // Use the lookAt matrix function to calculate the rotation of the particle
  // to face the camera.
  auto transform = Transform<float>(mat4f::lookAt(
      particle_instance.GetPosition(), emitter_info.GetCameraPosition(), kUp));
  particle_instance.SetRotation(transform.rotation);
}

}  // namespace imp
