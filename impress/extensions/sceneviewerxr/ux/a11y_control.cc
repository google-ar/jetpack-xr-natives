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

#include "extensions/sceneviewerxr/ux/a11y_control.h"

#include <algorithm>

#include "absl/time/time.h"
#include "filament/libs/math/include/math/TMatHelpers.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/subspace_root.h"
#include "vr/android_xr/sceneviewerxr/api/sceneviewerxr_session_listener.h"

namespace svxr {

namespace {

using float3 = ::imp::float3;
using float4 = ::imp::float4;
using mat4f = ::imp::mat4f;

constexpr auto kA11yControlIdleFadeOutDuration = absl::Milliseconds(120);
constexpr auto kA11yControlFadeInDuration = absl::Milliseconds(120);
constexpr auto kControlElevation = 0.025f;
constexpr auto kApproximateControlHalfHeight = 0.03f;
constexpr auto kYBufferAtMaxDepth = 0.5f;
constexpr float kTalkbackVisibleAlpha = 1.0f;

}  // namespace

A11yControl::~A11yControl() = default;

void A11yControl::Setup(SceneViewerXrSessionListener* session_listener,
                        android_xr::SubspaceRoot* subspace_root) {
  session_listener_ = session_listener;
  subspace_root_ = subspace_root;
  footprint_ = GetNode()->GetComponent<Footprint>();
}

void A11yControl::Cleanup() {}

void A11yControl::OnUpdate(const imp::FrameTime& delta_time,
                           const InteractionMode& interaction_data) {
  machine_.UpdateWithAlternatives(
      [](A11yControlStates::Initialized& state)
          -> A11yControlStates::Machine::OptionalState {
        return A11yControlStates::Machine::OptionalState{
            A11yControlStates::Hidden{}};
      },
      [this, interaction_data](A11yControlStates::Hidden& state)
          -> A11yControlStates::Machine::OptionalState {
        return UpdateHidden(state, interaction_data);
      },
      [this, interaction_data, delta_time](A11yControlStates::Active& state)
          -> A11yControlStates::Machine::OptionalState {
        return UpdateActive(state, interaction_data, delta_time);
      });
}

void A11yControl::OnStateChange(
    const A11yControlStates::Machine& machine,
    const A11yControlStates::Machine::State& current_state,
    const A11yControlStates::Machine::State& next_state) {}

A11yControlStates::Machine::OptionalState A11yControl::UpdateActive(
    A11yControlStates::Active& state, const InteractionMode& interaction_data,
    const imp::FrameTime& delta_time) {
  const bool talkback_enabled =
      session_listener_ && session_listener_->IsTalkbackEnabled();
  const bool model_is_selected =
      interaction_data.TestSelected(InteractionMode::SelectedMode::kSelected);
  if (!talkback_enabled || !model_is_selected || !GetNode()->IsEnabled()) {
    state.alpha.SetTarget(0.f, kA11yControlIdleFadeOutDuration);
  } else if (state.alpha.GetTarget() != kTalkbackVisibleAlpha) {
    state.alpha.SetTarget(kTalkbackVisibleAlpha, kA11yControlFadeInDuration);
  }
  UpdateTransformAndView();

  if (!state.alpha.IsAtTarget()) {
    state.alpha.Step(delta_time.GetDeltaTime());
  } else if (state.alpha.GetTarget() == 0.f) {
    // Enter hidden once we have completed an update at alpha of 0.
    return A11yControlStates::Machine::OptionalState{
        A11yControlStates::Hidden{}};
  }

  return {};
}

A11yControlStates::Machine::OptionalState A11yControl::UpdateHidden(
    A11yControlStates::Hidden& state, const InteractionMode& interaction_data) {
  const bool talkback_enabled =
      session_listener_ && session_listener_->IsTalkbackEnabled();
  const bool model_is_selected =
      interaction_data.TestSelected(InteractionMode::SelectedMode::kSelected);
  if (talkback_enabled && model_is_selected && GetNode()->IsEnabled()) {
    A11yControlStates::Active next_state;
    next_state.alpha.Setup(0.f);
    next_state.alpha.SetTarget(kTalkbackVisibleAlpha,
                               kA11yControlFadeInDuration);
    return A11yControlStates::Machine::OptionalState{next_state};
  }
  return {};
}

bool A11yControl::IsVisible() {
  return machine_.ApplyWithAlternatives(
      [](A11yControlStates::Initialized& state) -> float { return true; },
      [](A11yControlStates::Hidden& state) -> float { return false; },
      [](A11yControlStates::Active& state) -> float { return true; });
}

float A11yControl::GetAlpha() {
  return machine_.ApplyWithAlternatives(
      [](A11yControlStates::Initialized& state) -> float { return 0.f; },
      [](A11yControlStates::Hidden& state) -> float { return 0.f; },
      [](A11yControlStates::Active& state) -> float {
        return state.alpha.Get();
      });
}

void A11yControl::UpdateTransformAndView() {
  auto node = GetNode();
  footprint_ = node->GetComponent<Footprint>();
  if (!footprint_) {
    return;
  }
  auto camera = GetView().GetCameraManager().GetCamera();
  mat4f world_from_rig = node->GetWorldTrs();
  mat4f world_from_camera = camera->GetNode()->GetWorldTrs();
  float3 camera_position = (world_from_camera * imp::kZero3).xyz;
  // The rig position is at the dead center of the footprint, under the model.
  float3 rig_position = (world_from_rig * imp::kZero3).xyz;
  float3 rig_to_camera = camera_position - rig_position;
  float3 rig_to_camera_xz = normalize(rig_to_camera * float3(1.f, 0.f, 1.f));
  // Radius of a circle that encloses the footprint.
  float radius = length(footprint_->GetFootprintSize()) * 0.5f;

  float3 ideal_position;
  const float3 menu_panel_center_position =
      rig_position + rig_to_camera_xz * (radius + kFootprintSlop);
  if (type_ == A11yControlType::kScale) {
    // The scale button is positioned below the menu panel.
    const float3 down_vector = -normalize(world_from_camera[1].xyz);
    constexpr float kVerticalOffsetFromPanel = 0.08f;
    ideal_position =
        menu_panel_center_position + down_vector * kVerticalOffsetFromPanel;
  } else {
    // The rotation buttons are positioned relative to the menu panel.

    // Get a vector pointing to the right of the model, relative to camera.
    const float3 right_vector = normalize(cross(imp::kUp, rig_to_camera_xz));

    float3 direction;
    if (type_ == A11yControlType::kRotateLeft) {
      direction = -right_vector;
    } else {  // kRotateRight
      direction = right_vector;
    }

    // The approximate horizontal distance from the center of the menu panel to
    // the center of the accessibility control.
    constexpr float kHorizontalOffsetFromPanel = 0.15f;
    ideal_position =
        menu_panel_center_position + direction * kHorizontalOffsetFromPanel;
  }

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
    y_offset_addition = kYBufferAtMaxDepth * scale;
  } else if (current_distance >= kSystemBaseline) {
    scale = kScaleAboveBaseline + clamped_distance * increment;
    float y_offset_scale = kScaleAboveBaseline + kSystemMaximumDepth;
    y_offset_addition = kYBufferAtMaxDepth * y_offset_scale * increment;
  }

  // Shift the control down, taking distance scaling into account.
  ideal_position.y -= (kControlElevation + ((scale + y_offset_addition) *
                                            kApproximateControlHalfHeight));

  // The transform for the control (in world space).
  mat4f world_from_ideal_label = mat4f::lookAt(
      ideal_position, ideal_position - camera_to_ideal_position, imp::kUp);
  mat4f subspace_from_world = subspace_root_->GetSubspaceFromWorldTransform();
  // The transform for the control (in task space).
  ::imp::Transform<float> transform(subspace_from_world *
                                    world_from_ideal_label);

  if (session_listener_) {
    switch (type_) {
      case A11yControlType::kRotateLeft:
        session_listener_->OnA11yRotateLeftControlAnchorUpdated(
            transform.translation, transform.rotation, scale, GetAlpha());
        break;
      case A11yControlType::kRotateRight:
        session_listener_->OnA11yRotateRightControlAnchorUpdated(
            transform.translation, transform.rotation, scale, GetAlpha());
        break;
      case A11yControlType::kScale:
        session_listener_->OnA11yScaleControlAnchorUpdated(
            transform.translation, transform.rotation, scale, GetAlpha());
        break;
    }
  }
}

}  // namespace svxr
