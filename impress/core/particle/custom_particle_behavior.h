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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_CUSTOM_PARTICLE_BEHAVIOR_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_CUSTOM_PARTICLE_BEHAVIOR_H_

#include "core/particle/particle_behavior_result.h"
#include "core/particle/particle_instance.h"

namespace imp {

// Interface used to implement custom particle behavior. Users can implement
// a derived instance and pass it into the ParticleSystem during initialization.
// When particles are created and updated, the system will call the implemented
// methods to perform the custom behavior. This is structured as a class so that
// users may encapsulate any state needed by their behavior. Note that any state
// included here is shared by all particle instances. This object will be
// destroyed when the ParticleBehavior instance owning it is destroyed.
class CustomParticleBehavior {
 public:
  virtual ~CustomParticleBehavior() = default;

  // Called once when a particle is first created, after the default setup.
  virtual void OnInitialized(ParticleInstance& particle) {};

  // Called each frame on each particle after the default behavior is updated.
  virtual ParticleBehaviorResult OnUpdate(ParticleInstance& particle,
                                          float delta_time_seconds) {
    return ParticleBehaviorResult::kActive;
  };
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_CUSTOM_PARTICLE_BEHAVIOR_H_
