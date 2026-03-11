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

#include "extensions/sceneviewerxr/ux/menu_panel.h"

#include <algorithm>

#include "absl/time/time.h"
#include "filament/libs/math/include/math/TMatHelpers.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "core/math/mat.h"
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

constexpr auto kMenuPanelIdleFadeOutDuration = absl::Milliseconds(120);
constexpr auto kMenuPanelFadeInDuration = absl::Milliseconds(120);
constexpr auto kPanelElevation = 0.025f;
constexpr auto kApproximatePanelHalfHeight = 0.03f;
constexpr auto kYBufferAtMaxDepth = 0.5f;

bool IsVisibleBasedOnFlags(const InteractionMode& interaction_data) {
  // If any transformation is occurring, be sure to hide the panel.
  if (!interaction_data.TestTransform(
          InteractionMode::TransformMode::kNothing)) {
    return false;
  }

  // Apart from during transformation, the panel is always visible
  // during an object's 'Selected' state.
  if (interaction_data.TestSelected(InteractionMode::SelectedMode::kSelected)) {
    return true;
  }

  return false;
}

}  // namespace

MenuPanel::MenuPanel() : machine_(MenuPanelStates::Initialized{}, this) {}
MenuPanel::~MenuPanel() = default;

void MenuPanel::Setup(SceneViewerXrSessionListener* session_listener,
                      android_xr::SubspaceRoot* subspace_root) {
  session_listener_ = session_listener;
  subspace_root_ = subspace_root;
  footprint_ = GetNode()->GetComponent<Footprint>();
}

void MenuPanel::Cleanup() {}

void MenuPanel::OnUpdate(const imp::FrameTime& delta_time,
                         const InteractionMode& interaction_data) {
  machine_.UpdateWithAlternatives(
      [](MenuPanelStates::Initialized& state) -> Machine::OptionalState {
        return Machine::OptionalState{MenuPanelStates::Hidden{}};
      },
      [this, interaction_data](
          MenuPanelStates::Hidden& state) -> Machine::OptionalState {
        return UpdateHidden(state, interaction_data);
      },
      [this, interaction_data,
       delta_time](MenuPanelStates::Active& state) -> Machine::OptionalState {
        return UpdateActive(state, interaction_data, delta_time);
      });
}

void MenuPanel::OnStateChange(const Machine& machine,
                              const Machine::State& current_state,
                              const Machine::State& next_state) {}

MenuPanelStates::Machine::OptionalState MenuPanel::UpdateActive(
    MenuPanelStates::Active& state, const InteractionMode& interaction_data,
    const imp::FrameTime& delta_time) {
  if (!IsVisibleBasedOnFlags(interaction_data)) {
    state.alpha.SetTarget(0.f, kMenuPanelIdleFadeOutDuration);
  } else if (state.alpha.GetTarget() != 1.f) {
    state.alpha.SetTarget(1.f, kMenuPanelFadeInDuration);
  }
  UpdateTransformAndView();

  if (!state.alpha.IsAtTarget()) {
    state.alpha.Step(delta_time.GetDeltaTime());
  } else if (state.alpha.GetTarget() == 0.f) {
    // Enter hidden once we have completed an update at alpha of 0.
    return Machine::OptionalState{MenuPanelStates::Hidden{}};
  }

  return {};
}

MenuPanel::Machine::OptionalState MenuPanel::UpdateHidden(
    MenuPanelStates::Hidden& state, const InteractionMode& interaction_data) {
  if (IsVisibleBasedOnFlags(interaction_data)) {
    MenuPanelStates::Active next_state;
    next_state.alpha.Setup(0.f);
    next_state.alpha.SetTarget(1.f, kMenuPanelFadeInDuration);
    return Machine::OptionalState{next_state};
  }
  return {};
}

bool MenuPanel::IsVisible() {
  return machine_.ApplyWithAlternatives(
      [](MenuPanelStates::Initialized& state) -> float { return true; },
      [](MenuPanelStates::Hidden& state) -> float { return false; },
      [](MenuPanelStates::Active& state) -> float { return true; });
}

float MenuPanel::GetAlpha() {
  return machine_.ApplyWithAlternatives(
      [](MenuPanelStates::Initialized& state) -> float { return 0.f; },
      [](MenuPanelStates::Hidden& state) -> float { return 0.f; },
      [](MenuPanelStates::Active& state) -> float {
        return state.alpha.Get();
      });
}

void MenuPanel::UpdateTransformAndView() {
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
  float3 rig_to_camera_xz = rig_to_camera * float3(1.f, 0.f, 1.f);
  // Radius of a circle that encloses the footprint.
  float radius = length(footprint_->GetFootprintSize()) * 0.5f;

  // Points towards the camera in the plane of the footprint.
  float3 ideal_position =
      rig_position + normalize(rig_to_camera_xz) * (radius + kFootprintSlop);

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

  // Shift the panel down, taking distance scaling into account.
  ideal_position.y -= (kPanelElevation + ((scale + y_offset_addition) *
                                          kApproximatePanelHalfHeight));

  // The transform for the panel (in world space).
  mat4f world_from_ideal_label = mat4f::lookAt(
      ideal_position, ideal_position - camera_to_ideal_position, imp::kUp);
  mat4f subspace_from_world = subspace_root_->GetSubspaceFromWorldTransform();
  // The transform for the panel (in task space).
  ::imp::Transform<float> transform(subspace_from_world *
                                    world_from_ideal_label);

  if (session_listener_) {
    session_listener_->OnPanelAnchorUpdated(
        transform.translation, transform.rotation, scale, GetAlpha());
  }
}

}  // namespace svxr
