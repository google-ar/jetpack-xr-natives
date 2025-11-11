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

#include "core/particle/particle_instance.h"
#include "core/view/utils/frame_time.h"

namespace imp {

ParticleBehavior::UpdateResult ParticleBehavior::UpdateParticle(
    const FrameTime& frame_time, ParticleInstance& particle_instance) {
  // Lifetime is the most common reason a particle will expire, check it first.
  if (particle_instance.HasRemainingLifetimeSeconds()) {
    UpdateResult result = UpdateLifetime(frame_time, particle_instance);
    if (result != ParticleBehavior::UpdateResult::kActive) {
      return result;
    }
  }

  // TODO: (broken link) - Support movement behavior.
  // TODO: (broken link) - Support billboarding behavior.
  // TODO: (broken link) - Support scaling behavior.
  // TODO: (broken link) - Support alpha behavior.

  return UpdateResult::kActive;
}

ParticleBehavior::UpdateResult ParticleBehavior::UpdateLifetime(
    const FrameTime& frame_time, ParticleInstance& particle_instance) {
  // Update the particle's remaining lifetime.
  float seconds = particle_instance.GetRemainingLifetimeSeconds();
  seconds -= frame_time.GetDeltaSeconds();
  particle_instance.SetRemainingLifetimeSeconds(seconds);

  // If the lifetime has run out, notify the caller that this particle may now
  // be destroyed.
  if (seconds <= 0.0f) {
    return ParticleBehavior::UpdateResult::kExpired;
  }

  return ParticleBehavior::UpdateResult::kActive;
}

}  // namespace imp
