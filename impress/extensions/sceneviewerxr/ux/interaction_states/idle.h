/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_IDLE_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_IDLE_H_

#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"

namespace svxr {
namespace interaction_states {

Machine::OptionalState Update(Idle& state, const imp::FrameTime& delta_time,
                              InteractionOwner& owner);

Machine::OptionalState HandleInput(interaction_states::Idle& state,
                                   const imp::Ray& ray,
                                   imp::NodeHandle receiver,
                                   const imp::float3& hit_position,
                                   imp::Flags<InputFlag> input_flags,
                                   InteractionOwner& owner);

Machine::OptionalState ResetSize(interaction_states::Idle& state,
                                 InteractionOwner& owner);

Idle SetupIdleState();

}  // namespace interaction_states
}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_IDLE_H_
