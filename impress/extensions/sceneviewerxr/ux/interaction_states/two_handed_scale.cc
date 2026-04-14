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

#include "extensions/sceneviewerxr/ux/interaction_states/two_handed_scale.h"

#include <cmath>

#include "core/common/enum_flags.h"
#include "core/common/smooth.h"
#include "core/math/math.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"
#include "extensions/sceneviewerxr/ux/state_machine.h"

namespace svxr::interaction_states {

Machine::OptionalState Update(const imp::FrameTime& delta_time,
                              TwoHandedScale& state, InteractionOwner& owner) {
  float initial_pinch_distance =
      length(state.initial_world_space_ray_left.origin -
             state.initial_world_space_ray_right.origin);
  float current_pinch_distance =
      length(state.current_world_space_ray_left.origin -
             state.current_world_space_ray_right.origin);

  float current_scale = 1.0f;
  if (initial_pinch_distance > 1e-5f) {
    current_scale = current_pinch_distance / initial_pinch_distance;
  }
  if (std::abs(current_scale - 1.0f) > 1e-5f) {
    state.has_scaled = true;
  }

  float model_log_scale =
      std::log(std::exp(state.initial_model_log_scale) * current_scale);
  auto constrained_model_log_scale = owner.ConstrainElastically(
      model_log_scale, owner.GetModelLogScaleLimits(), kElasticScale);

  owner.GetModelLogScale().SetTarget(constrained_model_log_scale);
  return {};
}

Machine::OptionalState HandleInput(TwoHandedScale& state, const imp::Ray& ray,
                                   imp::NodeHandle receiver,
                                   imp::Flags<InputFlag> input_flags,
                                   InteractionOwner& owner) {
  if (input_flags.Test(InputFlag::kIsDownStopping)) {
    bool is_right_remaining = !input_flags.Test(InputFlag::kIsRight);
    const bool use_translation = is_right_remaining
                                     ? state.was_right_translation
                                     : state.was_left_translation;
    const auto& remaining_ray = is_right_remaining
                                    ? state.current_world_space_ray_right
                                    : state.current_world_space_ray_left;
    if (use_translation) {
      auto rig_node = owner.GetRigNode();
      auto head_position = owner.GetHeadPosition();
      auto world_space_rig_position = rig_node->GetWorldPosition();
      auto target_distance = length(world_space_rig_position - head_position);
      auto hand_distance = length(remaining_ray.origin - head_position);
      float initial_distance_ratio = 1.0f;
      if (hand_distance > 1e-5f) {
        initial_distance_ratio = target_distance / hand_distance;
      }

      return Machine::OptionalState{Translation{
          .initial_world_space_ray = remaining_ray,
          .current_world_space_ray = remaining_ray,
          // Since we scale equally from the center, the assumed hit position is
          // the rig center.
          .initial_world_space_rig_to_hit = imp::kZero3,
          .initial_world_space_hit_position = world_space_rig_position,
          .pickup_offset = imp::kZero3,
          .anchor_snap_position = world_space_rig_position,
          .rig_local_position =
              imp::Smooth<imp::float3>(kSmoothFastResolvingPositionParameters,
                                       rig_node->GetLocalPosition()),
          .is_right = is_right_remaining,
          .is_active = true,
          .cumulative_change_delta = 0,
          .initial_distance_ratio = initial_distance_ratio,
          .has_translated = true,  // Prevent unintentional snap since we
                                   // synthesized the starting point.
      }};
    }
    auto rig_rotation = owner.GetRigNode()->GetLocalRotation();
    return Machine::OptionalState{
        Rotation{.initial_world_space_ray = remaining_ray,
                 .current_world_space_ray = remaining_ray,
                 .initial_rig_rotation = rig_rotation,
                 .is_right = is_right_remaining,
                 .is_rotating_after_two_handed_scale = true,
                 .cumulative_change_delta = 0}};
  }
  owner.GetInteractionData().SetTransform(
      InteractionMode::TransformMode::kScale);
  (input_flags.Test(InputFlag::kIsRight) ? state.current_world_space_ray_right
                                         : state.current_world_space_ray_left) =
      ray;
  return {};
}

}  // namespace svxr::interaction_states
