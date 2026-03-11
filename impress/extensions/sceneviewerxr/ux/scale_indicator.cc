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

#include "extensions/sceneviewerxr/ux/scale_indicator.h"

#include <algorithm>

#include "absl/time/time.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/gltf_bounds.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/subspace_root.h"
#include "vr/android_xr/sceneviewerxr/api/sceneviewerxr_session_listener.h"

namespace svxr {

namespace {

using float3 = ::imp::float3;
using mat4f = ::imp::mat4f;
template <typename T>
using Transform = ::imp::Transform<T>;

constexpr auto kScaleIndicatorIdleFadeOutDuration = absl::Milliseconds(120);
constexpr auto kScaleIndicatorFadeInDuration = absl::Milliseconds(120);
constexpr auto kScaleIndicatorAdjustmentDuration = absl::Milliseconds(50);
constexpr auto kScaleIndicatorYOffsetDuration = absl::Milliseconds(500);
constexpr auto kScaleIndicatorFadeoutDelay = absl::Milliseconds(1000);
constexpr auto kScaleIndicatorElevationFraction = 1.15f;
constexpr auto kScaleIndicatorMinHeight = 0.15f;
constexpr auto kPanelElevation = 0.025f;
constexpr auto kApproximatePanelHalfHeight = 0.03f;

bool IsVisibleBasedOnFlags(const InteractionMode& interaction_data) {
  // Only show the scale indicator during a scaling transformation.
  if (interaction_data.TestTransform(InteractionMode::TransformMode::kScale)) {
    return true;
  }
  return false;
}

}  // namespace

ScaleIndicator::ScaleIndicator()
    : machine_(ScaleIndicatorStates::Initialized{}, this) {}
ScaleIndicator::~ScaleIndicator() = default;

void ScaleIndicator::Setup(imp::NodeHandle model_node,
                           SceneViewerXrSessionListener* session_listener,
                           android_xr::SubspaceRoot* subspace_root) {
  model_node_ = model_node;
  session_listener_ = session_listener;
  subspace_root_ = subspace_root;
}

void ScaleIndicator::Cleanup() {}

void ScaleIndicator::OnUpdate(const imp::FrameTime& delta_time,
                              const InteractionMode& interaction) {
  machine_.UpdateWithAlternatives(
      [this](
          ScaleIndicatorStates::Initialized& state) -> Machine::OptionalState {
        UpdateTransformAndView();
        return Machine::OptionalState{ScaleIndicatorStates::Hidden{}};
      },
      [this, interaction](
          ScaleIndicatorStates::Hidden& state) -> Machine::OptionalState {
        return UpdateHidden(state, interaction);
      },
      [this, interaction, delta_time](
          ScaleIndicatorStates::Active& state) -> Machine::OptionalState {
        return UpdateActive(state, interaction, delta_time);
      });
}

void ScaleIndicator::OnStateChange(const Machine& machine,
                                   const Machine::State& current_state,
                                   const Machine::State& next_state) {}

ScaleIndicatorStates::Machine::OptionalState ScaleIndicator::UpdateActive(
    ScaleIndicatorStates::Active& state, const InteractionMode& interaction,
    const imp::FrameTime& delta_time) {
  ScaleIndicatorStates::Machine::OptionalState next_state = {};
  if (interaction.TestTransform(InteractionMode::TransformMode::kScale)) {
    state.percentage.SetTarget(GetModelScalePercentage(),
                               kScaleIndicatorAdjustmentDuration);
  }
  // If the user has stopped scaling and the scale indicator is visible, setup a
  // hide delay and fade out.
  if (!IsVisibleBasedOnFlags(interaction) && state.alpha.GetTarget() != 0.0f) {
    state.hide_delay.Setup(0.f);
    state.hide_delay.SetTarget(1.f, kScaleIndicatorFadeoutDelay);
    state.alpha.SetTarget(0.f, kScaleIndicatorIdleFadeOutDuration);
  }

  state.percentage.Step(delta_time.GetDeltaTime());

  if (!state.hide_delay.IsAtTarget()) {
    state.hide_delay.Step(delta_time.GetDeltaTime());
  } else if (!state.alpha.IsAtTarget()) {
    state.alpha.Step(delta_time.GetDeltaTime());
  } else if (state.percentage.IsAtTarget() && state.alpha.GetTarget() == 0.f) {
    // Enter hidden once we have completed an update at alpha of 0.
    next_state = Machine::OptionalState{ScaleIndicatorStates::Hidden{}};
  }

  imp::Box current_bounds =
      model_node_->GetOrAddComponent<GltfBounds>()->GetLocalBounds();
  if (state.y_offset.GetTarget() != current_bounds.halfExtent.y) {
    state.y_offset.SetTarget(current_bounds.halfExtent.y,
                             kScaleIndicatorYOffsetDuration);
  }
  if (!state.y_offset.IsAtTarget()) {
    state.y_offset.Step(delta_time.GetDeltaTime());
  };

  UpdateTransformAndView();

  return next_state;
}

ScaleIndicator::Machine::OptionalState ScaleIndicator::UpdateHidden(
    ScaleIndicatorStates::Hidden& state, const InteractionMode& interaction) {
  ScaleIndicator::Machine::OptionalState next_state = {};
  if (IsVisibleBasedOnFlags(interaction)) {
    ScaleIndicatorStates::Active active_state;
    // No hide delay needed here (snap to whatever target it has).
    active_state.hide_delay.Snap();
    active_state.alpha.Setup(0.f);
    active_state.alpha.SetTarget(1.f, kScaleIndicatorFadeInDuration);
    active_state.percentage.Setup(GetModelScalePercentage());
    imp::Box current_bounds =
        model_node_->GetOrAddComponent<GltfBounds>()->GetLocalBounds();
    active_state.y_offset.Setup(current_bounds.halfExtent.y);
    next_state = active_state;
  }
  return next_state;
}

bool ScaleIndicator::IsVisible() {
  return machine_.ApplyWithAlternatives(
      [](ScaleIndicatorStates::Initialized& state) -> float { return true; },
      [](ScaleIndicatorStates::Hidden& state) -> float { return false; },
      [](ScaleIndicatorStates::Active& state) -> float { return true; });
}

float ScaleIndicator::GetModelScalePercentage() {
  float scale = imp::Transform<float>(model_node_->GetLocalTrs()).scale.x;
  return scale * 100.f;
}

float ScaleIndicator::GetScalePercentage() {
  return machine_.ApplyWithAlternatives(
      [](ScaleIndicatorStates::Initialized& state) -> float { return 0.f; },
      [](ScaleIndicatorStates::Hidden& state) -> float { return 0.f; },
      [](ScaleIndicatorStates::Active& state) -> float {
        return state.percentage.Get();
      });
}

float ScaleIndicator::GetAlpha() {
  return machine_.ApplyWithAlternatives(
      [](ScaleIndicatorStates::Initialized& state) -> float { return 0.f; },
      [](ScaleIndicatorStates::Hidden& state) -> float { return 0.f; },
      [](ScaleIndicatorStates::Active& state) -> float {
        return state.alpha.Get();
      });
}

float ScaleIndicator::GetYOffset() {
  return machine_.ApplyWithAlternatives(
      [](ScaleIndicatorStates::Initialized& state) -> float { return 0.f; },
      [](ScaleIndicatorStates::Hidden& state) -> float { return 0.f; },
      [](ScaleIndicatorStates::Active& state) -> float {
        return state.y_offset.Get();
      });
}

void ScaleIndicator::UpdateTransformAndView() {
  float alpha = GetAlpha();
  float scale_percentage = GetScalePercentage();
  auto node = GetNode();
  auto camera = GetView().GetCameraManager().GetCamera();
  mat4f world_from_rig = node->GetWorldTrs();
  mat4f rig_from_model = model_node_->GetLocalTrs();
  auto model_transform = Transform<float>(rig_from_model);
  mat4f world_from_camera = camera->GetNode()->GetWorldTrs();
  float3 camera_position = (world_from_camera * imp::kZero3).xyz;
  // The rig position is at the dead center of the footprint, under the model.
  float3 rig_position = (world_from_rig * imp::kZero3).xyz;
  float height = GetYOffset() * 2.f * model_transform.scale.y;
  height = std::max(height, kScaleIndicatorMinHeight);
  float3 ideal_position =
      rig_position + imp::kUp * height * kScaleIndicatorElevationFraction;

  // Calculate the size of UI based on the distance to the camera so it is
  // independent of it.
  float3 camera_to_ideal_position = camera_position - ideal_position;
  float current_distance = length(camera_to_ideal_position);

  float clamped_distance =
      std::clamp(current_distance, kSystemBaseline, kSystemMaximumDepth);
  float scale_factor =
      clamped_distance / (kSystemMaximumDepth - kSystemBaseline);

  float increment = std::clamp(kScaleIncrementRate * scale_factor, 0.0f, 1.0f);
  float scale = kScaleAtBaseline;
  float y_offset_addition = 0.0f;

  // We only partially apply the distance to the scale
  // if we are above the system-defined baseline depth.
  if (current_distance > kSystemMaximumDepth) {
    scale = kScaleAboveMaxDepth + current_distance * increment;
    y_offset_addition =
        kScaleIncrementRate * (current_distance - kSystemMaximumDepth);
  } else if (current_distance >= kSystemBaseline) {
    scale = kScaleAboveBaseline + clamped_distance * increment;
  }

  // Shift the panel up, taking distance scaling into account.
  ideal_position.y += (kPanelElevation + ((scale + y_offset_addition) *
                                          kApproximatePanelHalfHeight));

  // The transform for the scale indicator (in world space).
  mat4f world_from_ideal_label = mat4f::lookAt(
      ideal_position, ideal_position - camera_to_ideal_position, imp::kUp);
  mat4f subspace_from_world = subspace_root_->GetSubspaceFromWorldTransform();
  Transform<float> transform(subspace_from_world * world_from_ideal_label);
  if (session_listener_) {
    session_listener_->OnScaleIndicatorAnchorUpdated(transform.translation,
                                                     transform.rotation, scale,
                                                     alpha, scale_percentage);
  }
}

}  // namespace svxr
