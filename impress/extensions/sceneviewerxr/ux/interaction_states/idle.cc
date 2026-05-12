// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "extensions/sceneviewerxr/ux/interaction_states/idle.h"

#include <cmath>

#include "core/common/log.h"
#include "absl/time/time.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/common/smooth.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"
#include "extensions/sceneviewerxr/ux/ramp.h"

namespace svxr {
namespace interaction_states {

using SnapMode = Footprint::SnapMode;

constexpr auto kIdleTimeToUnselect = absl::Seconds(5);

Machine::OptionalState Update(Idle& state, const imp::FrameTime& delta_time,
                              InteractionOwner& owner) {
  // If we are just entering idle from an interaction state,
  // clear the flag and also set the pointer mode to nothing.
  // This allows the footprint to fade away.
  auto& interaction_data = owner.GetInteractionData();
  if (interaction_data.TestGaze(InteractionMode::GazeMode::kGaze) &&
      state.gaze_input_since_last_update) {
    interaction_data.SetPointer(InteractionMode::PointerMode::kNothing);
    if (state.gaze_input_since_last_update) {
      state.gaze_input_since_last_update = false;
    }
  }

  // When talkback is enabled, we don't want the idle timeout to run.
  // if (session_listener_ && session_listener_->IsTalkbackEnabled()) {
  if (owner.IsTalkbackEnabled()) {
    return {};
  }

  if (owner.IsIdleTimeoutEnabled() &&
      interaction_data.TestPointer(InteractionMode::PointerMode::kNothing)) {
    // Update the idle timer.
    state.inactive_time_counter.Step(delta_time.GetDeltaTime());
    if (state.inactive_time_counter.IsAtTarget()) {
      interaction_data.SetSelected(InteractionMode::SelectedMode::kUnselected);
      owner.GetFootprint()->SetColliderEnabled(false);
    }
  } else if (state.inactive_time_counter.Get() > 0) {
    // Reset the idle timer.
    state.inactive_time_counter.Setup(0);
    state.inactive_time_counter.SetTarget(1, kIdleTimeToUnselect);
  }
  return {};
}

Machine::OptionalState HandleInput(interaction_states::Idle& state,
                                   const imp::Ray& ray,
                                   imp::NodeHandle receiver,
                                   const imp::float3& hit_position,
                                   imp::Flags<InputFlag> input_flags,
                                   InteractionOwner& owner) {
  // Check if we are gazing; if yes, we want to manually track hovering.
  auto& interaction_data = owner.GetInteractionData();
  bool is_gaze = input_flags.Test(InputFlag::kIsGaze);
  interaction_data.SetGaze(is_gaze ? InteractionMode::GazeMode::kGaze
                                   : InteractionMode::GazeMode::kDefault);
  if (is_gaze) {
    state.gaze_input_since_last_update = true;
  }
  bool is_receiver_android_panel =
      !owner.ReceiverIsModel(receiver) && !owner.ReceiverIsFootprint(receiver);
  if (input_flags.Test(InputFlag::kIsDownStarting)) {
    interaction_data.SetPointer(InteractionMode::PointerMode::kPress);
    if (is_receiver_android_panel) {
      return {};
    }
    if (!owner.ReceiverInitiatesTranslation(receiver)) {
      // Start rotating.
      auto rig_node = owner.GetRigNode();
      auto rig_rotation = rig_node->GetLocalRotation();
      return Machine::OptionalState{interaction_states::Rotation{
          .initial_world_space_ray = ray,
          .current_world_space_ray = ray,
          .initial_rig_rotation = rig_rotation,
          .is_right = input_flags.Test(InputFlag::kIsRight),
          .is_rotating_after_two_handed_scale = false,
          .cumulative_change_delta = 0}};
    }

    // Start translating.
    auto rig_node = owner.GetRigNode();
    auto world_space_rig_position = rig_node->GetWorldPosition();

    auto head_position = owner.GetHeadPosition();
    auto target_distance = length(hit_position - head_position);
    auto hand_distance = length(ray.origin - head_position);

    auto pickup_offset =
        (owner.GetFootprint().IsValid() &&
         owner.GetFootprint()->IsSnapMode(SnapMode::kSnappedToPlane))
            ? imp::kUp * kPickupOffset
            : imp::kZero3;

    auto translation = interaction_states::Translation{
        .initial_world_space_ray = ray,
        .current_world_space_ray = ray,
        .initial_world_space_rig_to_hit =
            hit_position - world_space_rig_position,
        .initial_world_space_hit_position = hit_position,
        .pickup_offset = pickup_offset,
        .anchor_snap_position = rig_node->GetWorldPosition(),
        .rig_local_position =
            imp::Smooth<imp::float3>(kSmoothFastResolvingPositionParameters,
                                     rig_node->GetLocalPosition()),
        .is_right = input_flags.Test(InputFlag::kIsRight),
        .is_active = true,
        .cumulative_change_delta = 0,
        .initial_distance_ratio = target_distance / hand_distance};

    if (owner.GetFootprint().IsValid() &&
        owner.GetFootprint()->IsSnapMode(SnapMode::kSnappedToPlane)) {
      translation.anchor_cooldown.Setup(3.5f);
      translation.anchor_cooldown.SetTarget(0.f, absl::Seconds(.75f));
      translation.lift_cooldown.Setup(.25f);
      translation.lift_cooldown.SetTarget(0.f, absl::Seconds(.25f));
    } else {
      translation.anchor_cooldown.Setup(0.f);
      translation.anchor_cooldown.SetTarget(0.f, absl::ZeroDuration());
      translation.lift_cooldown.Setup(0.f);
      translation.lift_cooldown.SetTarget(0.f, absl::ZeroDuration());
      translation.anchor_cooldown.Snap();
      translation.lift_cooldown.Snap();
    }
    return Machine::OptionalState{translation};
  } else if (input_flags.Test(InputFlag::kIsHoverStarting)) {
    interaction_data.SetPointer(InteractionMode::PointerMode::kHover);
  } else if (input_flags.Test(InputFlag::kIsHoverStopping) ||
             input_flags.Test(InputFlag::kIsDownStopping)) {
    interaction_data.SetPointer(InteractionMode::PointerMode::kNothing);
  }
  return {};
}

Machine::OptionalState ResetSize(interaction_states::Idle& state,
                                 InteractionOwner& owner) {
  // Determine the final scale based on the reset scale type.
  float final_log_scale = owner.GetResetLogScale();

  auto result = interaction_states::ScaleReset{};
  auto model_node = owner.GetModelNode();

  auto model_transform = imp::Transform<float>(model_node->GetLocalTrs());

  result.initial_model_log_scale = std::log(model_transform.scale.x);
  result.final_model_log_scale = final_log_scale;

  auto& model_log_scale = owner.GetModelLogScale();

  model_log_scale.SetTarget(result.final_model_log_scale);

  IMP_LOG(imp::INFO) << "Resetting model size from "
            << std::exp(result.initial_model_log_scale) << " to "
            << std::exp(result.final_model_log_scale);

  owner.GetInteractionData().SetTransform(
      InteractionMode::TransformMode::kScale);

  result.minimum_display_duration.Setup(0);
  result.minimum_display_duration.SetTargetWithUnitDuration(1,
                                                            kResetMinDuration);

  owner.ResetRigPosition();

  owner.ToggleResetScaleType();

  return result;
}

Idle SetupIdleState() {
  Idle next_state{};
  next_state.inactive_time_counter.Setup(0);
  next_state.inactive_time_counter.SetTarget(1, kIdleTimeToUnselect);
  return next_state;
}

}  // namespace interaction_states
}  // namespace svxr
