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

#include "extensions/sceneviewerxr/ux/load_indicator_component.h"

#include <cmath>

#include "absl/time/time.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/subspace_root.h"
#include "vr/android_xr/sceneviewerxr/api/sceneviewerxr_session_listener.h"

namespace svxr {

using Machine = LoadIndicatorStates::Machine;

constexpr float kUserToLoader = 1.0f;
constexpr absl::Duration kPositionLerp = absl::Milliseconds(120);
constexpr absl::Duration kLoadIndicatorFadeoutDelay = absl::Milliseconds(1000);
constexpr absl::Duration kLoadIndicatorFadeoutDuration =
    absl::Milliseconds(120);

// Helper function to setup the loaded state (used in multiple places).
Machine::OptionalState SetupLoadedState() {
  LoadIndicatorStates::Loaded next_state;
  next_state.visibility_factor.Setup(1.0f);
  next_state.visibility_factor.SetTarget(0.0f, kLoadIndicatorFadeoutDuration);
  next_state.hide_delay.Setup(0);
  next_state.hide_delay.SetTarget(1.f, kLoadIndicatorFadeoutDelay);
  return next_state;
}

LoadIndicatorComponent::LoadIndicatorComponent()
    : machine_(LoadIndicatorStates::Connecting{}, this) {}
LoadIndicatorComponent::~LoadIndicatorComponent() = default;

void LoadIndicatorComponent::Setup(
    SceneViewerXrSessionListener* session_listener,
    android_xr::SubspaceRoot& subspace_root) {
  session_listener_ = session_listener;
  subspace_root_ = &subspace_root;

  Reset();
}

void LoadIndicatorComponent::Update(const imp::FrameTime& delta_time) {
  machine_.UpdateWithAlternatives(
      [this](LoadIndicatorStates::Initialize& state) -> Machine::OptionalState {
        LoadIndicatorStates::Connecting next_state;
        next_state.initial_download_size =
            GetView().GetAssetManager().GetDownloadedSize();
        // Ensure that the load indicator will be set at the correct position
        // when it is initialized.
        CalculateNodeTransform();
        lerped_node_position_.Snap();
        return next_state;
      },
      [this, delta_time](
          LoadIndicatorStates::Connecting& state) -> Machine::OptionalState {
        return UpdateConnecting(state, delta_time);
      },
      [this, delta_time](
          LoadIndicatorStates::Loading& state) -> Machine::OptionalState {
        return UpdateLoading(state, delta_time);
      },
      [this, delta_time](LoadIndicatorStates::Loaded& state)
          -> Machine::OptionalState { return UpdateLoaded(state, delta_time); },
      [](LoadIndicatorStates::Closed& state) -> Machine::OptionalState {
        return {};
      });
}

Machine::OptionalState LoadIndicatorComponent::UpdateConnecting(
    LoadIndicatorStates::Connecting& state, const imp::FrameTime& delta_time) {
  float download_progress = GetView().GetAssetManager().GetDownloadProgress(
      state.initial_download_size);
  if (download_progress >= 0) {
    LoadIndicatorStates::Loading next_state;
    next_state.initial_download_size = state.initial_download_size;
    UpdatePanel(delta_time, download_progress);
    return next_state;
  }
  UpdatePanel(delta_time, download_progress);
  return {};
}

Machine::OptionalState LoadIndicatorComponent::UpdateLoading(
    LoadIndicatorStates::Loading& state, const imp::FrameTime& delta_time) {
  float download_progress = GetView().GetAssetManager().GetDownloadProgress(
      state.initial_download_size);
  if (download_progress >= 1.0f) {
    auto next_state = SetupLoadedState();
    UpdatePanel(delta_time, 1);
    return next_state;
  }
  UpdatePanel(delta_time, download_progress);
  return {};
}

Machine::OptionalState LoadIndicatorComponent::UpdateLoaded(
    LoadIndicatorStates::Loaded& state, const imp::FrameTime& delta_time) {
  if (!state.hide_delay.IsAtTarget()) {
    state.hide_delay.Step(delta_time.GetDeltaTime());
  } else if (!state.visibility_factor.IsAtTarget()) {
    state.visibility_factor.Step(delta_time.GetDeltaTime());
    if (state.visibility_factor.IsAtTarget()) {
      session_listener_->OnCloseLoadIndicator();
      UpdatePanel(delta_time, 1, 0);
      return LoadIndicatorStates::Closed{};
    }
  }

  UpdatePanel(delta_time, 1, state.visibility_factor.Get());
  return {};
}

void LoadIndicatorComponent::UpdatePanel(const imp::FrameTime& delta_time,
                                         float download_progress, float alpha) {
  CalculateNodeTransform();

  if (!lerped_node_position_.IsAtTarget()) {
    lerped_node_position_.Step(delta_time.GetDeltaTime());
  }

  if (session_listener_) {
    session_listener_->OnLoadIndicatorAnchorUpdated(
        lerped_node_position_.Get(), node_transform_.rotation, 1.0f, alpha,
        download_progress);
  }
}

void LoadIndicatorComponent::Close() {
  machine_.UpdateWithAlternatives(
      [](LoadIndicatorStates::Closed& state) -> Machine::OptionalState {
        return {};
      },
      // Any state the machine is in should transition to the Loaded state.
      [](auto& state) -> Machine::OptionalState { return SetupLoadedState(); });
}

void LoadIndicatorComponent::Reset() {
  node_transform_ =
    imp::Transform<float>(imp::kZero3, imp::kIdentityQuatf, imp::kOne3);
  lerped_node_position_.Setup({0, 0, -kUserToLoader});

  machine_.UpdateWithAlternatives(
      // Any state the machine is in should transition to the Initialize state.
      [](auto& state) -> Machine::OptionalState {
        return LoadIndicatorStates::Initialize{};
      });
}

void LoadIndicatorComponent::CalculateNodeTransform() {
  auto camera = GetView().GetCameraManager().GetCamera();
  imp::mat4f world_from_camera = camera->GetNode()->GetWorldTrs();

  imp::float3 ideal_view_position = imp::float3(
      0, kMinSvNodeDistance * sinf(imp::ToRadians(-kViewDropDegrees)),
      -kMinSvNodeDistance * cosf(imp::ToRadians(-kViewDropDegrees)));
  imp::float3 ideal_world_position =
      (world_from_camera * ideal_view_position).xyz;
  imp::float3 camera_position = (world_from_camera * imp::kZero3).xyz;

  // Calculate the size of UI based on the distance to the camera so it is
  // independent of it.
  imp::float3 camera_to_ideal_position = camera_position - ideal_world_position;

  // The transform for the scale indicator (in world space).
  imp::mat4f world_from_ideal_label = imp::mat4f::lookAt(
      ideal_world_position, ideal_world_position - camera_to_ideal_position,
      imp::kUp);
  imp::mat4f subspace_from_world =
      subspace_root_->GetSubspaceFromWorldTransform();
  imp::Transform<float> transform(subspace_from_world * world_from_ideal_label);

  // Ensure that the look at is also applied to the node when UpdatePanel is
  // called.
  node_transform_.rotation = transform.rotation;

  lerped_node_position_.SetTarget(transform.translation, kPositionLerp);
}

}  // namespace svxr
