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

#ifndef THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_BEHAVIOR_RESULT_H_
#define THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_BEHAVIOR_RESULT_H_

namespace imp {

// Results from particle behavior updates. kActive is the general purpose ok
// result, meaning the particle is active and will continue processing. Any
// other result should be interpreted as a signal for the caller to take a
// defined action, such as destroying the particle (kExpired).
enum class ParticleBehaviorResult {
  // The operation was performed successfully, the particle remains active.
  kActive = 0,

  // The particle has expired, the particle should be destroyed. This may be
  // the result of a lifetime update when the particle lifetime has elapsed.
  kExpired,
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PARTICLE_PARTICLE_BEHAVIOR_RESULT_H_
