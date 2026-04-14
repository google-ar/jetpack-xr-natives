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
#include "core/common/enum_flags.h"
#include "core/math/math.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/idle.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"
#include "extensions/sceneviewerxr/ux/state_machine.h"

namespace svxr::interaction_states {
namespace {

constexpr imp::float3 kOneHandedScaleUnit = imp::float3(0.17f, 0.17f, 1.0f);
constexpr imp::float3 kOneHandedScaleDeadzone = imp::float3(0.05f);
constexpr float kOneHandedScaleThrow = 2.0f;
constexpr float kOneHandedScaleMultiplier = 1.0f;

constexpr float kElasticScale = 10.f;

}  // namespace

Machine::OptionalState Update(const imp::FrameTime& delta_time,
                              OneHandedScale& state, InteractionOwner& owner) {
  imp::mat4 world_from_camera =
      owner.GetCamera()->GetCamera()->getModelMatrix();
  imp::mat4 camera_from_world = inverse(world_from_camera);

  // Convert points to camera space.
  imp::float3 initial_camera_space_origin =
      (camera_from_world *
       imp::float4(state.initial_world_space_ray.origin, 1.0f))
          .xyz;
  imp::float3 current_camera_space_origin =
      (camera_from_world *
       imp::float4(state.current_world_space_ray.origin, 1.0f))
          .xyz;

  imp::float3 delta_translation =
      current_camera_space_origin - initial_camera_space_origin;
  // Ignore Z (depth) changes for the 1D scale.
  delta_translation.z = 0.0f;

  imp::float3 delta_translation_with_deadzone =
      greaterThan(delta_translation, kOneHandedScaleDeadzone) *
          (delta_translation - kOneHandedScaleDeadzone) +
      lessThan(delta_translation, -kOneHandedScaleDeadzone) *
          (delta_translation + kOneHandedScaleDeadzone);

  // Project delta strictly along the camera's Y axis (Up/Down in the camera
  // plane). This makes scaling up = dragging anywhere in the upper half (Up,
  // Up-Left, Up-Right) and down = dragging anywhere in the lower half (Down,
  // Down-Left, Down-Right).
  float scalar_delta_translation =
      delta_translation_with_deadzone.y / kOneHandedScaleUnit.y;

  float scale_offset =
      pow(fabs(scalar_delta_translation), kOneHandedScaleThrow) *
      ((scalar_delta_translation < 0.f) ? -1.f : 1.f) *
      kOneHandedScaleMultiplier;

  if (std::abs(scale_offset) > 1e-5f) {
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
