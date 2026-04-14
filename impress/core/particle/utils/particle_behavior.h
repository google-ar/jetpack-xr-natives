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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_BEHAVIOR_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_BEHAVIOR_H_

#include <memory>

#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/particle/custom_particle_behavior.h"
#include "core/particle/particle_behavior_result.h"
#include "core/particle/particle_emitter_info.h"
#include "core/particle/particle_emitter_state.proto.imp.h"
#include "core/particle/particle_instance.h"

namespace imp::imp_particle {

// This class is a collection of functions that perform specific behavioral
// updates on a particle. It will own any state data needed by a ParticleSystem
// to perform behaviors on particles. For example, target range values, or
// curves. It is also where users will be able to register custom behaviors.
// There are implementations for common particle behaviors, such as lifetime and
// position, as well as a generic update method that inspects the particle and
// performs all supported behaviors.
class ParticleBehavior {
 public:
  // Initializes the object by caching any relevant behavior data, including
  // creating objects, such as curves, needed to update particles over time.
  ParticleBehavior(
      NodeHandle emitter_node, const ParticleConfig& particle_config,
      std::unique_ptr<CustomParticleBehavior> custom_particle_behavior =
          std::unique_ptr<CustomParticleBehavior>());

  // Initializes a ParticleInstance with appropriate default values for all
  // fields used by the particle.
  void SetDefaultValues(ParticleInstance& particle_instance);

  // Performs all behaviors intrinsic to the particle. Returns kActive if all
  // behaviors were performed and the particle remains active, or one of the
  // other ParticleBehaviorResults if other actions should be taken.
  ParticleBehaviorResult UpdateParticle(
      const imp::ParticleEmitterInfo& emitter_info, float delta_seconds,
      ParticleInstance& particle_instance);

 protected:
  // Updates the time a particle will remain active. Returns kActive if the
  // lifetime update was performed successfully, or kExpired if the particle's
  // lifetime has elapsed and it should be destroyed.
  //
  // Callers are required to verify the ParticleInstance has the lifetime field
  // before calling this method.
  ParticleBehaviorResult UpdateLifetime(float delta_seconds,
                                        ParticleInstance& particle_instance);

  // Updates the position and velocity of a particle. Does not return an
  // ParticleBehaviorResult because movement does not presently cause a particle
  // to expire.
  //
  // Callers are required to verify the ParticleInstance has the velocity field
  // before calling this method.
  void UpdateMovement(float delta_seconds, ParticleInstance& particle_instance);

  // Updates the orientation of a particle to face the camera. Does not return
  // an ParticleBehaviorResult because billboard behavior does not presently
  // cause a particle to expire.
  //
  // Callers are required to verify the ParticleInstance has the rotation field
  // before calling this method.
  void UpdateBillboard(const imp::ParticleEmitterInfo& emitter_info,
                       ParticleInstance& particle_instance);

 private:
  NodeHandle emitter_node_;
  std::unique_ptr<CustomParticleBehavior> custom_particle_behavior_;
  float default_lifetime_seconds_ = 0.0f;
  float3 default_velocity_ = kZero3;
  float3 default_acceleration_ = kZero3;
  float3 default_scale_ = kOne3;
  quatf default_rotation_ = kIdentityQuatf;
  bool billboard_ = false;
};

}  // namespace imp::imp_particle

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_UTILS_PARTICLE_BEHAVIOR_H_
