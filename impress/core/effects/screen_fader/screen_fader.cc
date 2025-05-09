// Copyright 2024 Google LLC
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

#include "core/effects/screen_fader/screen_fader.h"

#include "absl/status/status.h"
#include "core/common/smooth.h"
#include "core/effects/screen_fader/screen_fader_assets.h"
#include "core/ncsb/component_handle.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/framework/render/primitive_shape_renderer.h"
#include "core/view/framework/render/primitive_shape_renderer_state.proto.imp.h"

namespace imp {

constexpr absl::string_view kColorMaterialParameter = "color";
constexpr absl::string_view kAlphaMaterialParameter = "alpha";

// Set a high acceleration limit, this enables us to use Smooth to linearly
// interpolate over an exact time in a consistent way.
constexpr float kSmoothAccelerationLimit = 1000.0f;

Future<absl::Status> ScreenFader::Setup(ScreenFader::Params params) {
  // Setup the alpha smooth field so that it steps at the correct rate to
  // animate the fade over the specified duration.
  params.duration_seconds = std::max(params.duration_seconds, 0.001f);
  SmoothParameters smooth_params = SmoothParameters(
      1.0f / params.duration_seconds, kSmoothAccelerationLimit);
  alpha_.Setup(smooth_params, 0.0f);

  // Setup the color parameter.
  MaterialDefinition::Parameter color_param;
  color_param.name = std::string(kColorMaterialParameter);
  *color_param.mutable_float3_val() = params.color;

  // Setup the quad that holds the fader.
  PrimitiveShapeRendererState primitive_shape_state;
  primitive_shape_state.primitive = {
      .material =
          MaterialDefinition{
              .asset = std::string(
                  screen_fader::kScreenFaderMaterialCmat.GetIdentifier()),
              .parameters = {color_param}},
      .mesh = PrimitiveShapeRendererState::QuadMesh{}};
  primitive_shape_state.frustrum_culling_mode =
      PrimitiveShapeRendererState::FrustrumCullingMode::DISABLED;
  primitive_shape_state.shadow_casting_mode =
      PrimitiveShapeRendererState::ShadowMode::NONE;
  primitive_shape_state.shadow_receiving_mode =
      PrimitiveShapeRendererState::ShadowMode::NONE;
  primitive_shape_state.priority = 0;

  return GetNode()
      ->AddComponentWithState<PrimitiveShapeRenderer>(primitive_shape_state)
      .Then([this](ComponentHandle<PrimitiveShapeRenderer> primitive_renderer) {
        mesh_renderer_ = GetNode()->GetComponent<MeshRenderer>();
      });
}

void ScreenFader::Update(const FrameTime& frame_time) {
  // Updates the alpha based on the elapsed time.
  if (state_ == State::kFadingIn || state_ == State::kFadingOut) {
    alpha_.Step(frame_time.GetDeltaSeconds());
    mesh_renderer_->GetMaterial()->SetParameter(
        std::string(kAlphaMaterialParameter), alpha_.Get());

    if (alpha_.Get() == alpha_.GetTarget()) {
      SetState(state_ == State::kFadingIn ? State::kFadedIn : State::kFadedOut);
    }
  }
}

void ScreenFader::FadeOut() {
  if (state_ == State::kFadedOut || state_ == State::kFadingOut) {
    return;
  }

  alpha_.SetTarget(1.0f);
  SetState(State::kFadingOut);
}

void ScreenFader::FadeIn() {
  if (state_ == State::kFadedIn || state_ == State::kFadingIn) {
    return;
  }

  alpha_.SetTarget(0.0f);
  SetState(State::kFadingIn);
}

ScreenFader::State ScreenFader::GetState() const { return state_; }

void ScreenFader::SetState(State state) {
  // Return early, the state hasn't changed.
  if (state == state_) {
    return;
  }

  State previous_state = state_;
  state_ = state;

  GetNode()->Send(StateChangedEvent(state_, previous_state, GetHandle(this)));
}

}  // namespace imp
