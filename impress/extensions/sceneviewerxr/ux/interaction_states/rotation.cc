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

#include "extensions/sceneviewerxr/ux/interaction_states/rotation.h"

#include <algorithm>
#include <cmath>

#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/idle.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"

namespace svxr {
namespace interaction_states {

using float3 = ::imp::float3;
using float4 = ::imp::float4;
using quatf = ::imp::quatf;

namespace {
constexpr auto kMinimumRotationDelta = .5f;
}  // namespace

Machine::OptionalState Update(Rotation& state, const imp::FrameTime& delta_time,
                              InteractionOwner& owner) {
  state.active_duration += delta_time.GetDeltaTime();
  imp::mat4 world_from_camera =
      owner.GetCamera()->GetCamera()->getModelMatrix();
  imp::mat4 camera_from_world = inverse(world_from_camera);

  float rotation_turntable_angle = 0.f;
  float rotation_scaling_delta = 0.f;
  {
    // Project the initial and current ray direction into the XZ and YZ planes.
    float3 initial_direction_world = state.initial_world_space_ray.direction;
    float3 current_direction_world = state.current_world_space_ray.direction;

    float3 initial_direction =
        (camera_from_world * float4(initial_direction_world, 0.f)).xyz;
    float3 current_direction =
        (camera_from_world * float4(current_direction_world, 0.f)).xyz;

    float3 initial_ray_xz =
        initial_direction - dot(initial_direction, imp::kUp);
    float3 current_ray_xz =
        current_direction - dot(current_direction, imp::kUp);
    float3 initial_ray_yz =
        initial_direction - dot(initial_direction, imp::kRight);
    float3 current_ray_yz =
        current_direction - dot(current_direction, imp::kRight);

    float dot_xz = dot(initial_ray_xz, current_ray_xz);
    float dot_yz = dot(initial_ray_yz, current_ray_yz);
    float determinant_xz = current_ray_xz.x * initial_ray_xz.z -
                           current_ray_xz.z * initial_ray_xz.x;
    float determinant_yz = initial_ray_yz.y * current_ray_yz.z -
                           initial_ray_yz.z * current_ray_yz.y;
    float angle_xz = atan2(determinant_xz, dot_xz);
    float angle_yz = atan2(determinant_yz, dot_yz);

    constexpr auto kRotationTurntableAmplificationScale = -1.0f;
    rotation_turntable_angle = kRotationTurntableAmplificationScale * angle_xz;

    constexpr auto kRotationScaleAmplificationScale = 0.07f;
    constexpr auto kRotationScalingDeadzoneSize = 0.125f;
    float adjusted_angle_yz =
        angle_yz > 0 ? std::max(0.f, angle_yz - kRotationScalingDeadzoneSize)
                     : std::min(0.f, angle_yz + kRotationScalingDeadzoneSize);
    rotation_scaling_delta =
        kRotationScaleAmplificationScale * adjusted_angle_yz;
  }

  float translation_turntable_angle = 0.f;
  float translation_scaling_delta = 0.f;
  {
    // Project the delta translation into the XY plane.
    float3 delta_translation_world = state.current_world_space_ray.origin -
                                     state.initial_world_space_ray.origin;
    float3 delta_translation =
        (camera_from_world * float4(delta_translation_world, 0.f)).xyz;
    float3 delta_translation_xy =
        delta_translation - dot(delta_translation, imp::kForward);

    constexpr auto kTranslationTurntableAmplificationScale = 4.0f;
    translation_turntable_angle =
        kTranslationTurntableAmplificationScale * delta_translation_xy.x;

    constexpr auto kTranslationScaleAmplificationScale = 2.0f;
    constexpr auto kTranslationScaleDeadzoneSize = 0.1f;
    float adjusted_delta_translation =
        delta_translation_xy.y > 0
            ? std::max(0.f,
                       delta_translation_xy.y - kTranslationScaleDeadzoneSize)
            : std::min(0.f,
                       delta_translation_xy.y + kTranslationScaleDeadzoneSize);
    translation_scaling_delta =
        kTranslationScaleAmplificationScale * adjusted_delta_translation;
  }

  float turntable_angle =
      rotation_turntable_angle + translation_turntable_angle;
  state.cumulative_change_delta += turntable_angle;

  if (state.cumulative_change_delta > kMinimumRotationDelta ||
      state.cumulative_change_delta < -kMinimumRotationDelta) {
    state.has_rotated = true;
  }

  auto turntable_rotation = quatf::fromAxisAngle(imp::kUp, turntable_angle);
  owner.SetRigRotationTarget(state.initial_rig_rotation * turntable_rotation);

  return {};
}

Machine::OptionalState HandleInput(Rotation& state, const imp::Ray& ray,
                                   imp::NodeHandle receiver,
                                   imp::Flags<InputFlag> input_flags,
                                   InteractionOwner& owner) {
  if (state.is_right != input_flags.Test(InputFlag::kIsRight)) {
    // Handle events for pointers which did not initiate rotation.
    if (input_flags.Test(InputFlag::kIsDownStarting)) {
      // Start two-handed scale.
      owner.GetInteractionData().SetScaleHandle(ScaleHandle::kTwoHanded);
      auto model_scale = owner.GetModelNode()->GetLocalScale().x;
      constexpr auto kEpsilon = 1e-5f;
      auto model_log_scale = std::log(std::max(kEpsilon, model_scale));

      auto& ray_right = state.is_right ? state.current_world_space_ray : ray;
      auto& ray_left = state.is_right ? ray : state.current_world_space_ray;
      return Machine::OptionalState{TwoHandedScale{
          .initial_world_space_ray_right = ray_right,
          .initial_world_space_ray_left = ray_left,
          .current_world_space_ray_right = ray_right,
          .current_world_space_ray_left = ray_left,
          .initial_model_log_scale = model_log_scale,
          .was_right_translation =
              state.is_right ? false
                             : owner.ReceiverInitiatesTranslation(receiver),
          .was_left_translation =
              state.is_right ? owner.ReceiverInitiatesTranslation(receiver)
                             : false,
      }};
    }
    // Ignore all other events for the other pointer.
    return {};
  }

  if (input_flags.Test(InputFlag::kIsDownStopping)) {
    if (state.cumulative_change_delta > kMinimumRotationDelta ||
        state.cumulative_change_delta < -kMinimumRotationDelta) {
      owner.GetInteractionData().SetTransform(
          InteractionMode::TransformMode::kNothing);
    } else if (!state.is_rotating_after_two_handed_scale &&
               owner.ReceiverIsModel(receiver)) {
      // Toggle select if we are not performing two-handed scale.
      bool is_footprint_enabled = owner.GetInteractionData().ToggleSelect();
      owner.GetFootprint()->SetColliderEnabled(is_footprint_enabled);
    }
  }

  if (!input_flags.Test(InputFlag::kIsDown)) {
    owner.GetInteractionData().SetTransform(
        InteractionMode::TransformMode::kNothing);
    return Machine::OptionalState{SetupIdleState()};
  } else {
    state.current_world_space_ray = ray;
  }

  if (state.cumulative_change_delta > kMinimumRotationDelta ||
      state.cumulative_change_delta < -kMinimumRotationDelta) {
    owner.GetInteractionData().SetTransform(
        InteractionMode::TransformMode::kRotate);
  }
  return {};
}

}  // namespace interaction_states
}  // namespace svxr
