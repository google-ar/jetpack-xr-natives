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

#include "extensions/sceneviewerxr/ux/interaction_states/scale_reset.h"

#include "core/common/smooth.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/idle.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"

namespace svxr {
namespace interaction_states {

Machine::OptionalState Update(const imp::FrameTime& delta_time,
                              ScaleReset& state, InteractionOwner& owner) {
  // If we are resetting to initial scale, ResetRigPosition has set the target
  // position explicitly. Don't call RequestUpdateRigPositionFromCamera on that
  // case or the rig_position target could be overridden.
  if (owner.GetResetScaleType() != ResetScaleType::kInitialScale) {
    owner.RequestUpdateRigPositionFromCamera(
        svxr::kSmoothFastResolvingPositionParameters);
  }

  // Note that since scale is updated after state machines, this state is exited
  // the frame after reaching unit scale.
  bool stopping =
      owner.GetModelLogScale().IsAtTarget() &&
      owner.GetModelLogScale().Get() == state.final_model_log_scale &&
      state.minimum_display_duration.IsAtTarget();

  if (stopping) {
    owner.GetInteractionData().SetTransform(
        InteractionMode::TransformMode::kNothing);
    return Machine::OptionalState{SetupIdleState()};
  }

  state.minimum_display_duration.Step(delta_time.GetDeltaTime());

  return {};
}

}  // namespace interaction_states
}  // namespace svxr
