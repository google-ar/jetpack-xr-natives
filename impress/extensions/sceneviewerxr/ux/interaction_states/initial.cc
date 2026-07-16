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

#include "extensions/sceneviewerxr/ux/interaction_states/initial.h"

#include <algorithm>
#include <cmath>
#include <variant>

#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/gltf_bounds.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"

namespace svxr {
namespace interaction_states {

namespace {

void FrameModel(imp::NodeHandle sv_node, const imp::mat4& camera_from_world,
                const imp::mat4& world_from_camera,
                const filament::Box& model_local_bounds,
                imp::float3* out_rig_position, float* out_model_scale) {
  imp::float3 sv_node_pos = sv_node->GetWorldPosition();
  float sv_node_view_distance =
      -static_cast<float>((camera_from_world * sv_node_pos).z);
  auto ideal_view_position =
      imp::float3(0,
                  -std::max(sv_node_view_distance, kMinSvNodeDistance) *
                      sinf(imp::ToRadians(kViewDropDegrees)),
                  -std::max(sv_node_view_distance, kMinSvNodeDistance) *
                      cosf(imp::ToRadians(kViewDropDegrees)));
  auto turn_table_radius = std::max(model_local_bounds.halfExtent.x,
                                    model_local_bounds.halfExtent.z);
  auto turn_table_radius_limit = -ideal_view_position.z * kViewRatio;
  auto vertical_radius = model_local_bounds.halfExtent.y;
  auto vertical_radius_limit = -ideal_view_position.z * kViewRatio;

  auto ideal_world_position = (world_from_camera * ideal_view_position).xyz;

  const float kMinModelSize = 0.01f;

  auto vertical_scale =
      vertical_radius_limit / std::max(vertical_radius, kMinModelSize);
  auto turn_table_scale =
      turn_table_radius_limit / std::max(turn_table_radius, kMinModelSize);
  // Use the smaller of the two choices.
  float ideal_scale = std::min(vertical_scale, turn_table_scale);

  *out_rig_position =
      (ideal_world_position) -
      (ideal_scale * imp::float3(0.f, model_local_bounds.halfExtent.y, 0.f));
  *out_model_scale = ideal_scale;
}

}  // namespace

Machine::OptionalState HandleInput(interaction_states::Initialized& state,
                                   InteractionOwner& owner) {
  bool is_idle_ready = owner.GetFootprint().IsValid() &&
                       owner.GetCamera().IsValid() &&
                       owner.GetModelNode().IsValid();
  if (!is_idle_ready) {
    // Defer entering Idle until our UX is ready.
    return {};
  }
  Opening opening_state;
  opening_state.minimum_duration.Setup(0.0f);
  opening_state.minimum_duration.SetTarget(1.0f, kOpeningAnimationDuration);
  return Machine::OptionalState{opening_state};
}

void OnStateEnd(const interaction_states::Machine::State& current_state,
                InteractionOwner& owner) {
  if (std::holds_alternative<interaction_states::Initialized>(current_state)) {
    // When leaving initialized, frame the model (we will actually be attached
    // to our parent at this point.) Determine and apply the initial framing.
    if (owner.GetCamera().IsValid() && owner.GetModelNode().IsValid()) {
      auto scene_viewer_node = owner.GetRigNode();
      imp::mat4 world_from_camera =
          owner.GetCamera()->GetCamera()->getModelMatrix();
      imp::mat4 camera_from_world = inverse(world_from_camera);
      imp::float3 rig_position = imp::kZero3;

      imp::Box local_bounds = owner.GetModelNode()
                                  ->GetOrAddComponent<GltfBounds>()
                                  ->GetLocalBounds();
      float model_scale = 1.0f;

      FrameModel(scene_viewer_node, camera_from_world, world_from_camera,
                 local_bounds, &rig_position, &model_scale);
      // Store the initial model scale and distance to camera for future reset.
      owner.SetInitialModelScale(model_scale);
      imp::float3 camera_position = (world_from_camera * imp::kZero3).xyz;
      owner.SetInitialModelDistanceToCamera(
          length(rig_position - camera_position));

      owner.CalculateModelScaleLimits();

      owner.SetModelLogScale(kSmoothSlowPopInScaleParameters, std::log(0.001f));
      owner.GetModelLogScale().SetTarget(
          std::log(owner.GetInitialModelScale()));

      owner.GetRigNode()->SetWorldPosition(rig_position);

      owner.SetRigPosition(kSmoothFastResolvingPositionParameters,
                           rig_position);
      owner.SetRigRotation(kSmoothRotationParameters, imp::kIdentityQuatf);

      owner.SetModelScale(model_scale);

      if (owner.GetUiEventListener() &&
          owner.GetUiEventListener()->IsTalkbackEnabled()) {
        owner.GetUiEventListener()->OnModelScaleChanged(model_scale);
      }
      owner.GetModelNode()->SetEnabled(true);

      if (owner.GetFootprint().IsValid()) {
        owner.GetFootprint()->OnInteractionMachineInitialized();
        // The scale pop-in animation takes ~300ms to visually settle.
        // We match this duration so the footprint fades in concurrently.
        owner.GetFootprint()->Show(kOpeningAnimationDuration);
      }
    }

      if (!(std::get<interaction_states::Initialized>(current_state)
                .is_reinitializing)) {
        if (owner.GetUiEventListener()) {
          owner.GetUiEventListener()->OnStartup();
        }
      }
    }
}

}  // namespace interaction_states
}  // namespace svxr
