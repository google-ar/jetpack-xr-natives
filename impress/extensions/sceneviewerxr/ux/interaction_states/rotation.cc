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
#include "extensions/sceneviewerxr/ux/footprint.h"
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
using SnapMode = Footprint::SnapMode;

namespace {
constexpr float kMinimumRotationDelta = 0.5f;
constexpr float kFreeFormRotationScale = 8.0f;

// Calculates the hand's translation delta projected onto a vertical plane
// aligned with the camera's view direction. This ensures that real-world
// horizontal movement only drives Yaw, and vertical movement drives Pitch.
float3 CalculateHandTranslationInVerticalCameraPlane(
    const imp::Ray& initial_ray, const imp::Ray& current_ray,
    const imp::mat4& world_from_camera) {
  float3 delta_translation_world = current_ray.origin - initial_ray.origin;
  float3 camera_right_in_world =
      (world_from_camera * float4(imp::kRight, 0.f)).xyz;

  return float3(dot(delta_translation_world, camera_right_in_world),
                dot(delta_translation_world, imp::kUp), 0.f);
}

// Converts 2D hand movement into 3D rotation angles (Yaw and Pitch).
void CalculateRotationAngles(const float3& delta_translation_xy,
                             float& turntable_angle, float& pitch_angle) {
  // Use pure translation to normalize rotation and remove directional tension.
  turntable_angle = kFreeFormRotationScale * delta_translation_xy.x;

  // Add a deadzone for the vertical axis to allow easier Y-only rotation.
  constexpr float kPitchDeadzone = 0.02f;  // 2cm
  float adjusted_y = 0.f;
  if (std::abs(delta_translation_xy.y) > kPitchDeadzone) {
    adjusted_y = delta_translation_xy.y > 0
                     ? delta_translation_xy.y - kPitchDeadzone
                     : delta_translation_xy.y + kPitchDeadzone;
  }
  pitch_angle = -kFreeFormRotationScale * adjusted_y;
}

}  // namespace

Machine::OptionalState Update(Rotation& state, const imp::FrameTime& delta_time,
                              InteractionOwner& owner) {
  state.active_duration += delta_time.GetDeltaTime();

  imp::mat4 world_from_camera =
      owner.GetCamera()->GetCamera()->getModelMatrix();

  float3 delta_translation_xy = CalculateHandTranslationInVerticalCameraPlane(
      state.initial_world_space_ray, state.current_world_space_ray,
      world_from_camera);

  if (!state.has_initialized_smoothing) {
    state.smoothed_delta_translation_xy = delta_translation_xy;
    state.has_initialized_smoothing = true;
  } else {
    constexpr float kSmoothingFactor = 0.8f;
    state.smoothed_delta_translation_xy =
        state.smoothed_delta_translation_xy * kSmoothingFactor +
        delta_translation_xy * (1.0f - kSmoothingFactor);
  }

  float turntable_angle = 0.f;
  float pitch_angle = 0.f;
  CalculateRotationAngles(state.smoothed_delta_translation_xy, turntable_angle,
                          pitch_angle);

  state.cumulative_change_delta += turntable_angle;
  state.turntable_angle = turntable_angle;
  state.pitch_angle = pitch_angle;

  if (state.cumulative_change_delta > kMinimumRotationDelta ||
      state.cumulative_change_delta < -kMinimumRotationDelta) {
    state.has_rotated = true;
  }

  bool is_anchored = false;
  if (owner.GetFootprint().IsValid()) {
    is_anchored = owner.GetFootprint()->IsSnapMode(SnapMode::kSnappedToPlane);
  }

  if (is_anchored) {
    quatf turntable_rotation = quatf::fromAxisAngle(imp::kUp, turntable_angle);
    // Multiply on the left to match world-space application.
    owner.SetRigRotationTarget(turntable_rotation * state.initial_rig_rotation);
  } else {
    float3 camera_right_in_world =
        (world_from_camera * float4(imp::kRight, 0.f)).xyz;

    quatf turntable_rotation = quatf::fromAxisAngle(imp::kUp, turntable_angle);
    quatf pitch_rotation =
        quatf::fromAxisAngle(camera_right_in_world, pitch_angle);

    // Multiply on the left to apply rotations in world/camera space.
    // Multiplying on the right would apply them in local space, which
    // depends on the model's current orientation.
    owner.SetRigRotationTarget(pitch_rotation * turntable_rotation *
                               state.initial_rig_rotation);
  }

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
      float model_scale = owner.GetModelNode()->GetLocalScale().x;
      constexpr float kEpsilon = 1e-5f;
      float model_log_scale = std::log(std::max(kEpsilon, model_scale));

      const imp::Ray& ray_right =
          state.is_right ? state.current_world_space_ray : ray;
      const imp::Ray& ray_left =
          state.is_right ? ray : state.current_world_space_ray;
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

    bool is_anchored = false;
    if (owner.GetFootprint().IsValid()) {
      is_anchored = owner.GetFootprint()->IsSnapMode(SnapMode::kSnappedToPlane);
    }

    if (!is_anchored) {
      quatf turntable_rotation =
          quatf::fromAxisAngle(imp::kUp, state.turntable_angle);
      // Multiply on the left to match world-space application in Update.
      owner.SetRigRotationTarget(turntable_rotation *
                                 state.initial_rig_rotation);
    }

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
