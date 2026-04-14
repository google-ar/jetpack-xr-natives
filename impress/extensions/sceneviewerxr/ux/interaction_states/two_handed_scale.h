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

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_TWO_HANDED_SCALE_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_TWO_HANDED_SCALE_H_

#include "core/collision/ray.h"
#include "core/math/vec.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"
#include "extensions/sceneviewerxr/ux/state_machine.h"

namespace svxr {
namespace interaction_states {

Machine::OptionalState Update(const imp::FrameTime& delta_time,
                              TwoHandedScale& state, InteractionOwner& owner);

Machine::OptionalState HandleInput(TwoHandedScale& state, const imp::Ray& ray,
                                   imp::NodeHandle receiver,
                                   imp::Flags<InputFlag> input_flags,
                                   InteractionOwner& owner);

}  // namespace interaction_states
}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_TWO_HANDED_SCALE_H_
