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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_SYSTEM_H_

#include <memory>

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/particle/custom_particle_behavior.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/utils/particle_emitter.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// The ParticleSystem component manages the lifecycle and resources needed for
// most typical particle systems. It is configured with a ParticleEmitterState
// proto.
class ParticleSystem : public Component {
 public:
  // Initializes the particle system with the given state and custom particle
  // behavior. If no custom particle behavior is provided, a null pointer is
  // provided to the emitter, signaling that no customization is required.
  Future<absl::Status> SetupWithState(
      std::unique_ptr<CustomParticleBehavior> custom_particle_behavior =
          std::unique_ptr<CustomParticleBehavior>());

  // Updates the particle system. This will be called once per frame by the
  // component lifecycle.
  void Update(const FrameTime& frame_time);

  // Returns the current particles per second count. This is the number of
  // particles, on average, that are being created per second.
  float GetEmissionRate() const;

  // Sets a new particles per second value. The emitter will attempt to create
  // the number of requested particles each second. This is limited by the
  // maximum number of particles requested when the system was initialized. This
  // value must be strictly > 0.0f. If you wish to pause emission of new
  // particles, please see SetEmissionPaused().
  void SetEmissionRate(float particles_per_second);

  // Returns the paused state of particle emission. See SetEmissionPaused for
  // more detail about this state. This will return true until the emitter has
  // finished initializing.
  bool IsEmissionPaused() const;

  // Sets whether the system's active particle emission is paused.
  //
  // When `pause` is true, the system acts as a gate and suspends emission of
  // any new particles. When `pause` is false, particles are emitted as normal.
  //
  // This does not affect existing particles. Emission remains limited by the
  // maximum particle count requested when the system was initialized.
  void SetEmissionPaused(bool pause);

  // Returns the custom material used by the particle system, specified in the
  // ParticleConfig.
  //
  // If no material was specified, a null pointer is returned.
  //
  // The returned pointer is owned by the ParticleSystem and should not be held
  // beyond the scope of the calling function.
  BorrowedMaterialPtr GetMaterial() const {
    return emitter_ ? emitter_->GetMaterial() : BorrowedMaterialPtr();
  }

  // Returns the current default lifetime for new particles. This value is
  // measured in seconds.
  float GetDefaultParticleLifetime() const {
    return emitter_ ? emitter_->GetDefaultParticleLifetime() : 0.0f;
  }

  // Sets the default lifetime for particles. If the particle system was not
  // initialized with a lifetime, this value will be retained, but ignored. The
  // new value will apply to all newly created particles and will not affect
  // existing particles. The value must be strictly greater than 0.0f. To stop
  // emission of new particles use the particle system's `SetEmissionPaused()`
  // method instead.
  void SetDefaultParticleLifetime(float default_lifetime_seconds) {
    if (emitter_) {
      emitter_->SetDefaultParticleLifetime(default_lifetime_seconds);
    }
  }

 private:
  ParticleEmitterState state_;
  imp_particle::OwnedParticleEmitterPtr emitter_;

 public:
  using IsfInfo = IsfInfo<&ParticleSystem::state_>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_SYSTEM_H_
