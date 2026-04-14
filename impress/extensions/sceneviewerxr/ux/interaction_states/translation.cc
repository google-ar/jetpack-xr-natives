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

#include "extensions/sceneviewerxr/ux/interaction_states/translation.h"

#include <algorithm>
#include <cmath>
#include <optional>

#include "absl/time/time.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/common/smooth.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/idle.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"

namespace svxr {
namespace interaction_states {

namespace {
using float3 = ::imp::float3;
using mat4f = ::imp::mat4f;
using SnapMode = Footprint::SnapMode;

constexpr auto kPlaneEpsilon = 1e-3f;
constexpr auto kMinimumTranslationDelta = .002f;
constexpr auto kDistanceRatioDeltaScale = 1.0f;
constexpr auto kMinAllowedModelToCameraDistanceDuringTranslation = 0.15f;

// Ray origin/forward are in world space.
mat4f GetRayFromWorldSpace(const imp::Ray& ray) {
  auto world_from_ray =
      mat4f::lookAt(ray.origin, ray.origin + ray.direction, imp::kUp);

  return inverse(world_from_ray);
}

}  // namespace

Machine::OptionalState Update(const imp::FrameTime& delta_time,
                              Translation& state, InteractionOwner& owner) {
  imp::NodeHandle component_node = owner.GetRigNode()->GetParent();

  state.active_duration += delta_time.GetDeltaTime();

  bool is_in_lift_mode = state.lift_cooldown.Get() > 0.f;
  if (is_in_lift_mode) {
    state.lift_cooldown.Step(delta_time.GetDeltaTime());
    if (state.anchor_snap_position.has_value()) {
      state.anchor_snap_position = std::nullopt;
    }
  }

  bool is_in_cooldown = state.anchor_cooldown.Get() > 0.f;
  if (is_in_cooldown) {
    state.anchor_cooldown.Step(delta_time.GetDeltaTime());
  }

  imp::ComponentHandle<svxr::Footprint> footprint = owner.GetFootprint();

  if (is_in_lift_mode) {
    if (footprint->IsSnapMode(SnapMode::kSnappedToPlane)) {
      owner.PlayLiftSound();
    }
    footprint->SetSnapMode(SnapMode::kLiftingOffPlane);
  } else if (is_in_cooldown) {
    footprint->SetSnapMode(SnapMode::kCooldown);
  } else if (footprint->IsSnapMode(SnapMode::kLiftingOffPlane) ||
             footprint->IsSnapMode(SnapMode::kCooldown)) {
    footprint->SetSnapMode(SnapMode::kNone);
  }

  if (state.is_active) {
    mat4f initial_ray_from_world =
        GetRayFromWorldSpace(state.initial_world_space_ray);
    mat4f current_ray_from_world =
        GetRayFromWorldSpace(state.current_world_space_ray);
    float3 initial_ray_relative_hit_position =
        (initial_ray_from_world * state.initial_world_space_hit_position).xyz;
    float3 current_world_space_hit_position =
        (inverse(current_ray_from_world) * initial_ray_relative_hit_position)
            .xyz;

    float3 origin_delta = state.current_world_space_ray.origin -
                          state.initial_world_space_ray.origin;
    float3 target_delta = origin_delta * (state.initial_distance_ratio - 1.f);

    current_world_space_hit_position += target_delta * kDistanceRatioDeltaScale;

    float3 current_world_space_rig_position =
        current_world_space_hit_position - state.initial_world_space_rig_to_hit;
    float3 target_position =
        component_node->LocalFromWorldPoint(current_world_space_rig_position);

    float3 footprint_target_position = imp::kZero3;
    if (owner.IsPassthrough()) {
      footprint_target_position = owner.ComputeFootprintPositionFromPlanes(
          current_world_space_hit_position,
          state.initial_world_space_rig_to_hit);

      if (!is_in_cooldown && !is_in_lift_mode) {
        state.anchor_snap_position =
            owner.GetAnchorSnapPosition(footprint_target_position);
      } else {
        state.anchor_snap_position = std::nullopt;
      }

      if (state.anchor_snap_position.has_value()) {
        footprint->SetSnapMode(SnapMode::kSnappable);
      } else if (footprint->IsSnapMode(SnapMode::kSnappable)) {
        footprint->SetSnapMode(SnapMode::kNone);
      }
    } else if (state.anchor_snap_position.has_value()) {
      state.anchor_snap_position = std::nullopt;
    }
    if (is_in_lift_mode) {
      target_position +=
          (state.pickup_offset * delta_time.GetDeltaSeconds() * 4.0f);
    } else {
      target_position += state.pickup_offset;
    }
    state.rig_local_position.SetTarget(target_position);
  }

  // Compare the distance between the camera and the rig.
  imp::mat4 world_from_camera =
      owner.GetCamera()->GetCamera()->getModelMatrix();
  float3 camera_world_position = (world_from_camera * imp::kZero3).xyz;
  float3 camera_world_position_xz =
      (camera_world_position)*float3(1.f, 0.f, 1.f);
  float3 rig_world_position =
      (owner.GetRigNode()->GetWorldTrs() * imp::kZero3).xyz;
  float3 rig_world_position_xz = (rig_world_position)*float3(1.f, 0.f, 1.f);
  float3 rig_to_camera_xz = camera_world_position_xz - rig_world_position_xz;

  float current_distance_to_camera = length(rig_to_camera_xz);

  state.rig_local_position.Step(delta_time.GetDeltaSeconds());

  float next_distance_to_camera =
      length(camera_world_position_xz -
             state.rig_local_position.Get() * imp::float3(1.f, 0.f, 1.f));

  float footprint_radius = length(footprint->GetFootprintSize()) * 0.5f;
  float minimum_distance_to_camera =
      footprint_radius + kMinAllowedModelToCameraDistanceDuringTranslation +
      kFootprintSlop;

  // If camera would be closer than before and closer than minimum distance, we
  // stop translating.
  imp::NodeHandle rig_node = owner.GetRigNode();
  if (next_distance_to_camera < minimum_distance_to_camera &&
      next_distance_to_camera < current_distance_to_camera) {
    state.rig_local_position.Setup(kSmoothSlowResolvingPositionParameters,
                                   rig_node->GetLocalPosition());
    rig_node->SetLocalPosition(state.rig_local_position.Get());
  } else if (footprint->IsSnapMode(SnapMode::kLiftingOffPlane)) {
    state.rig_local_position.SetParameters(kSoftAnchorPositionParameters);
  } else if (footprint->IsSnapMode(SnapMode::kSnappingToPlane)) {
    state.rig_local_position.SetParameters(
        kSmoothSlowResolvingPositionParameters);
  } else {
    state.rig_local_position.SetParameters(
        kSmoothFastResolvingPositionParameters);
  }

  float delta =
      distance(rig_node->GetLocalPosition(), state.rig_local_position.Get());
  state.cumulative_change_delta += delta;
  owner.GetRigPosition().SetTarget(state.rig_local_position.Get());

  // Check if we are currently lifting the model off of a plane.
  if (state.rig_local_position.IsAtTarget() &&
      footprint->IsSnapMode(SnapMode::kLiftingOffPlane)) {
    state.rig_local_position.SetParameters(
        kSmoothFastResolvingPositionParameters);
  }


  return {};
}

Machine::OptionalState HandleInput(Translation& state, const imp::Ray& ray,
                                   imp::NodeHandle receiver,
                                   imp::Flags<InputFlag> input_flags,
                                   InteractionOwner& owner) {
  if (!state.is_active) {
    // Ignore all events if the state is not active.
    return {};
  }

  // Handle events for pointers which did not initiate translation.
  if (state.is_right != input_flags.Test(InputFlag::kIsRight)) {
    if (input_flags.Test(InputFlag::kIsDownStarting)) {
      // Start two-handed scale.
      float model_scale = owner.GetModelNode()->GetLocalScale().x;
      constexpr auto kEpsilon = 1e-5f;
      auto model_log_scale = std::log(std::max(kEpsilon, model_scale));

      const imp::Ray& ray_right =
          state.is_right ? state.current_world_space_ray : ray;
      const imp::Ray& ray_left =
          state.is_right ? ray : state.current_world_space_ray;
      bool was_right_translation =
          state.is_right ? true : owner.ReceiverInitiatesTranslation(receiver);
      bool was_left_translation =
          state.is_right ? owner.ReceiverInitiatesTranslation(receiver) : true;

      return Machine::OptionalState{TwoHandedScale{
          .initial_world_space_ray_right = ray_right,
          .initial_world_space_ray_left = ray_left,
          .current_world_space_ray_right = ray_right,
          .current_world_space_ray_left = ray_left,
          .initial_model_log_scale = model_log_scale,
          .was_right_translation = was_right_translation,
          .was_left_translation = was_left_translation,
      }};
    }
    // Ignore all other events from other pointers.
    return {};
  }

  if (input_flags.Test(InputFlag::kIsDownStopping)) {
    owner.GetInteractionData().SetPointer(
        InteractionMode::PointerMode::kNothing);
    if (owner.GetInteractionData().TestTransform(
            InteractionMode::TransformMode::kTranslate)) {
      owner.GetInteractionData().SetTransform(
          InteractionMode::TransformMode::kNothing);
    } else if (owner.ReceiverIsModel(receiver)) {
      imp::ComponentHandle<svxr::Footprint> footprint = owner.GetFootprint();
      bool is_snapping = footprint->IsSnapMode(SnapMode::kSnappable) ||
                         footprint->IsSnapMode(SnapMode::kSnappingToPlane);

      // Only toggle selection if we are not snapping. Snapping implies intended
      // placement.
      if (!is_snapping) {
        bool is_footprint_enabled = owner.GetInteractionData().ToggleSelect();
        if (owner.IsTalkbackEnabled()) {
          owner.GetUiEventListener()->OnModelSelected(is_footprint_enabled);
        }
        footprint->SetColliderEnabled(is_footprint_enabled);
      }
    }
    state.rig_local_position.SetTarget(owner.GetRigPosition().Get());
    owner.GetRigPosition().SetTarget(owner.GetRigPosition().Get());
    imp::ComponentHandle<svxr::Footprint> footprint = owner.GetFootprint();
    if (footprint->IsSnapMode(SnapMode::kSnappingToPlane) ||
        footprint->IsSnapMode(SnapMode::kSnappable)) {
      owner.PlayDropSound();
      footprint->SetSnapMode(SnapMode::kSnappedToPlane);
      // Keep the model selected so the menu panel remains visible.
      if (owner.IsTalkbackEnabled()) {
        owner.GetUiEventListener()->OnModelSelected(true);
      }
      footprint->SetColliderEnabled(true);
      return Machine::OptionalState{SetupIdleState()};
    }
    return Machine::OptionalState{SetupIdleState()};
  }

  if (!input_flags.Test(InputFlag::kIsDown)) {
    owner.GetInteractionData().SetTransform(
        InteractionMode::TransformMode::kNothing);

    // Check to see if our footprint is offset (indicating the model should
    // now interpolate to match that offset).
    imp::NodeHandle rig_node = owner.GetRigNode();
    imp::ComponentHandle<svxr::Footprint> footprint = owner.GetFootprint();
    if (state.anchor_snap_position.has_value()) {
      float delta = length(state.anchor_snap_position.value() -
                           rig_node->GetLocalPosition());
      if (delta > kPlaneEpsilon) {
        state.rig_local_position.Setup(kSoftAnchorPositionParameters,
                                       rig_node->GetLocalPosition());
        state.rig_local_position.SetTarget(rig_node->GetLocalPosition() -
                                           delta);
        footprint->SetSnapMode(SnapMode::kSnappingToPlane);
      }
    }

    if (state.rig_local_position.IsAtTarget()) {
      if (footprint->IsSnapMode(SnapMode::kSnappingToPlane) ||
          footprint->IsSnapMode(SnapMode::kSnappable)) {
        owner.PlayDropSound();
        footprint->SetSnapMode(SnapMode::kSnappedToPlane);
        // Keep the model selected so the menu panel remains visible.
        if (owner.IsTalkbackEnabled()) {
          owner.GetUiEventListener()->OnModelSelected(true);
        }
        footprint->SetColliderEnabled(true);
        return Machine::OptionalState{SetupIdleState()};
      }
      return Machine::OptionalState{SetupIdleState()};
    } else {
      // Disable further input, don't exit state until we're at target.
      state.is_active = false;
    }
  }
  state.current_world_space_ray = ray;

  if (state.is_active &&
      state.cumulative_change_delta > kMinimumTranslationDelta) {
    owner.GetInteractionData().SetTransform(
        InteractionMode::TransformMode::kTranslate);
    state.has_translated = true;
  }

  return {};
}

}  // namespace interaction_states
}  // namespace svxr
