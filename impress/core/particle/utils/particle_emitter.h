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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_EMITTER_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_EMITTER_H_

#include <memory>
#include <string>

#include "core/common/owned_ptr.h"
#include "core/ncsb/node_handle.h"
#include "core/particle/custom_particle_behavior.h"
#include "core/particle/particle_emitter_info.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/utils/particle_behavior.h"
#include "core/particle/utils/particle_pool.h"
#include "core/view/utils/frame_time.h"

namespace imp::imp_particle {

// Interface for particle emitters. A particle emitter is responsible for
// for creating and managing particles within a particle system for the
// lifetime of the system. Impress supports node-based and instanced-based
// particles.
class ParticleEmitter {
 public:
  virtual ~ParticleEmitter() = default;

  // Updates all active particles in the system, creates and destroys particles
  // as defined by the ParticleEmitterConfig.
  virtual void UpdateParticleSystem(const FrameTime& frame_time) = 0;

  // Verifies elements of the emitter state to ensure the particle system can
  // be properly initialized.
  static std::string ValidateEmitterState(
      const ParticleEmitterState& emitter_state);

  // Returns the current particles per second count. This is the number of
  // particles, on average, that are being created per second.
  virtual float GetEmissionRate() const { return particles_per_second_; }

  // Sets a new particles per second value. The emitter will attempt to create
  // the number of requested particles each second. This is limited by the
  // maximum number of particles requested when the system was initialized. This
  // value must be strictly > 0.0f. If you wish to pause emission of new
  // particles, please see SetEmissionPaused().
  virtual void SetEmissionRate(float particles_per_second);

  // Returns the paused state of particle emission. See SetEmissionPaused for
  // more detail about this state.
  virtual bool IsEmissionPaused() const { return particle_emission_paused_; }

  // Sets whether the system's active particle emission is paused.
  //
  // When `pause` is true, the system acts as a gate and suspends emission of
  // any new particles. When `pause` is false, particles are emitted as normal.
  //
  // This does not affect existing particles. Emission remains limited by the
  // maximum particle count requested when the system was initialized.
  virtual void SetEmissionPaused(bool pause) {
    particle_emission_paused_ = pause;
  }

 protected:
  ParticleEmitter(
      NodeHandle emitter_node, const ParticleEmitterState& emitter_state,
      std::unique_ptr<CustomParticleBehavior> custom_particle_behavior);

  // Performs updates that effect the emitter itself, such as it's own lifetime.
  virtual void UpdateEmitterBehavior(const FrameTime& frame_time);

  // Determines if particles may be emitted. This will check different factors
  // such as the current particle delay, number of active particles, and the
  // Emitter lifetime. It does not emit a particle. The return value should not
  // be used to determine if the Emitter is active.
  virtual bool CanEmitParticles();

  // Returns information about the emitter used to update particle behavior.
  virtual imp::ParticleEmitterInfo GetParticleEmitterInfo() const;

 protected:
  // Reference to the emitter node.
  NodeHandle emitter_node_;

  // Holds data for all particles.
  ParticlePool particle_pool_;

  // Behaviors to perform behaviors on particles.
  ParticleBehavior particle_behavior_;

  // Emitter lifetime.
  bool emitter_duration_finite_ = false;
  float remaining_emitter_duration_ = 0.0f;
  float emitter_duration_ = 0.0f;
  bool looping_ = false;

  // Particle emission control.
  bool particle_emission_paused_ = false;
  float particles_per_second_ = 1.0f;
  float particle_delay_ = 0.0f;
};

// OwnedPtr and BorrowedPtr definitions for ParticleEmitter.
using OwnedParticleEmitterPtr = OwnedPtr<ParticleEmitter>;
using BorrowedParticleEmitterPtr = BorrowedPtr<ParticleEmitter>;

}  // namespace imp::imp_particle

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_EMITTER_H_
