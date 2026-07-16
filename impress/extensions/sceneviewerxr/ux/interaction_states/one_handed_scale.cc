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

#include "extensions/sceneviewerxr/ux/interaction_states/one_handed_scale.h"

#include <cmath>
#include <optional>

#include "core/camera/camera_component.h"
#include "core/collision/collision_helpers.h"
#include "core/common/enum_flags.h"
#include "core/math/math.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/gltf_bounds.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/idle.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"
#include "extensions/sceneviewerxr/ux/state_machine.h"

namespace svxr::interaction_states {
namespace {

constexpr float kElasticScale = 10.f;
constexpr float kMinDistanceForDivision = 1e-3f;
constexpr float kMinScaleOffsetToMarkScaled = 1e-5f;

}  // namespace

Machine::OptionalState Update(const imp::FrameTime& delta_time,
                              OneHandedScale& state, InteractionOwner& owner) {
  imp::NodeHandle footprint_node = owner.GetFootprintNode();
  if (!footprint_node) {
    return {};
  }

  imp::ComponentHandle<imp::CameraComponent> camera = owner.GetCamera();
  if (!camera) {
    return {};
  }

  imp::mat4f footprint_world_trs = footprint_node->GetWorldTrs();
  imp::float3 plane_point = footprint_node->GetWorldPosition();
  // Extracts the Y column (up vector) from the footprint's world transform.
  imp::float3 plane_normal = normalize(footprint_world_trs[1].xyz);

  // Get model center in world space and project onto plane.
  imp::NodeHandle model_node = owner.GetModelNode();
  imp::float3 center = plane_point;  // Fallback to footprint center.
  auto get_local_center =
      [](imp::NodeHandle node) -> std::optional<imp::float3> {
    imp::ComponentHandle<GltfBounds> gltf_bounds =
        node->GetComponent<GltfBounds>();
    if (gltf_bounds) {
      return gltf_bounds->GetLocalBounds().center;
    }
    imp::ComponentHandle<imp::GltfRenderer> gltf_renderer =
        node->GetComponent<imp::GltfRenderer>();
    if (gltf_renderer) {
      return gltf_renderer->GetLocalBounds().center;
    }
    return std::nullopt;
  };

  if (model_node) {
    std::optional<imp::float3> local_center_opt = get_local_center(model_node);
    if (local_center_opt) {
      imp::float3 world_center =
          (model_node->GetWorldTrs() * imp::float4(*local_center_opt, 1.0f))
              .xyz;
      // Project world center onto plane.
      float distance_to_plane = dot(world_center - plane_point, plane_normal);
      center = world_center - distance_to_plane * plane_normal;
    }
  }

  // Project initial ray onto plane to find handle proxy.
  std::optional<float> initial_t = imp::collision::RayIntersectPlane(
      state.initial_world_space_ray, plane_normal, plane_point);
  if (!initial_t.has_value()) {
    return {};
  }
  imp::float3 initial_intersection =
      state.initial_world_space_ray.origin +
      initial_t.value() * state.initial_world_space_ray.direction;

  // Project points to camera clip space.
  std::optional<imp::float3> center_clip_opt =
      camera->ClipFromWorldPoint(center);
  std::optional<imp::float3> handle_clip_opt =
      camera->ClipFromWorldPoint(initial_intersection);
  std::optional<imp::float3> hand_0_clip_opt =
      camera->ClipFromWorldPoint(state.initial_world_space_ray.origin);
  std::optional<imp::float3> hand_t_clip_opt =
      camera->ClipFromWorldPoint(state.current_world_space_ray.origin);

  if (!center_clip_opt.has_value() || !handle_clip_opt.has_value() ||
      !hand_0_clip_opt.has_value() || !hand_t_clip_opt.has_value()) {
    return {};  // Fallback or ignore if behind camera.
  }

  imp::float2 center_clip = imp::float2(center_clip_opt->x, center_clip_opt->y);
  imp::float2 handle_clip = imp::float2(handle_clip_opt->x, handle_clip_opt->y);
  imp::float2 hand_0_clip = imp::float2(hand_0_clip_opt->x, hand_0_clip_opt->y);
  imp::float2 hand_t_clip = imp::float2(hand_t_clip_opt->x, hand_t_clip_opt->y);

  imp::float2 dir_clip = handle_clip - center_clip;
  float dir_len = length(dir_clip);
  if (dir_len < kMinDistanceForDivision) {
    return {};  // Avoid division by zero.
  }
  imp::float2 dir_norm = dir_clip / dir_len;

  imp::float2 v_hand_0 = hand_0_clip - center_clip;
  imp::float2 v_hand_t = hand_t_clip - center_clip;

  float initial_signed_dist = dot(v_hand_0, dir_norm);
  float current_signed_dist = dot(v_hand_t, dir_norm);

  // Delta movement along handle direction.
  float delta = current_signed_dist - initial_signed_dist;

  // Map delta to log scale offset.
  // Sensitivity factor mapping clip space movement to log scale.
  constexpr float kScaleSensitivity = 1.5f;
  float scale_offset = delta * kScaleSensitivity;

  if (std::abs(scale_offset) > kMinScaleOffsetToMarkScaled) {
    state.has_scaled = true;
  }

  float model_log_scale = state.initial_model_log_scale + scale_offset;
  float constrained_model_log_scale = owner.ConstrainElastically(
      model_log_scale, owner.GetModelLogScaleLimits(), kElasticScale);
  owner.GetModelLogScale().SetTarget(constrained_model_log_scale);
  return {};
}

Machine::OptionalState HandleInput(OneHandedScale& state, const imp::Ray& ray,
                                   imp::NodeHandle receiver,
                                   imp::Flags<InputFlag> input_flags,
                                   InteractionOwner& owner) {
  // Ignore events for pointers which did not initiate one handed scale.
  if (state.is_right != input_flags.Test(InputFlag::kIsRight)) {
    return {};
  }

  if (!input_flags.Test(InputFlag::kIsDown)) {
    owner.GetInteractionData().SetTransform(
        InteractionMode::TransformMode::kNothing);
    owner.GetInteractionData().SetScaleHandle(std::nullopt);
    owner.PlayReleaseSound();
    return Machine::OptionalState{SetupIdleState()};
  } else {
    // If we are still down, we are potentially scaling.
    owner.GetInteractionData().SetTransform(
        InteractionMode::TransformMode::kScale);
    state.current_world_space_ray = ray;
  }
  return {};
}

}  // namespace svxr::interaction_states
