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

#include "core/editor/components/world_space_editor_ui.h"

#include <cfloat>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/actions/action_config.h"
#include "core/actions/controller_events.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/editor/components/world_space_editor_ui_assets.h"
#include "core/editor/editor.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/framework/render/primitive_shape_renderer.h"
#include "core/view/framework/render/primitive_shape_renderer_state.proto.imp.h"
#include "core/window/filament_host.h"

namespace imp::editor {

Future<absl::Status> WorldSpaceEditorUi::Setup(float2 texture_resolution) {
#if !IMP_RUNTIME(DEV)
  IMP_LOG(imp::FATAL) << "WorldSpaceEditorUi can only be used in dev mode.";
#endif

  window::FilamentHost::DevModeExtension* dev_mode_extension =
      GetView().GetHost()->TryGetExtension();
  if (dev_mode_extension && dev_mode_extension->HasRenderTarget()) {
    return Future<absl::Status>(
        absl::FailedPreconditionError("Can't create a WorldSpaceEditorUi when "
                                      "DevModeExtension already has an "
                                      "existing render target."));
  }

  if (texture_resolution.y <= 0 || texture_resolution.x <= 0) {
    return Future<absl::Status>(absl::InvalidArgumentError(
        "Invalid resolution. Both X and Y values must be positive."));
  }
  texture_resolution_ = texture_resolution;
  texture_aspect_ratio_ = texture_resolution.y / texture_resolution_.x;

  // Create and register a texture to act as a canvas for the ImGui UI.
  texture_ = GetView().GetTextureFactory().CreateTexture(
      texture_resolution.x, texture_resolution.y,
      filament::Texture::InternalFormat::RGBA8,
      filament::Texture::Usage::COLOR_ATTACHMENT |
          filament::Texture::Usage::SAMPLEABLE);

  // Listen for ControllerHitEvents on the Editor dispatcher
  Dispatcher& editor_dispatcher =
      GetView().GetRegistry().Get<editor::Editor>()->get().GetDispatcher();
  editor_dispatcher.Connect(
      [this](const ControllerHitEvent& event) mutable {
        HandleControllerHitEvent(event);
      },
      this);

  PrimitiveShapeRendererState primitive_shape_state;
  primitive_shape_state.primitive = {
      .material =
          MaterialDefinition{
              .asset = std::string(
                  world_space_editor_ui_assets::kWorldSpaceEditorUiTextureCmat
                      .GetIdentifier())},
      .mesh = PrimitiveShapeRendererState::QuadMesh{
          .size = float2{1.0f, texture_aspect_ratio_}, .flip_uv = true}};

  return GetNode()
      ->AddComponentWithState<PrimitiveShapeRenderer>(primitive_shape_state)
      .Then([this](ComponentHandle<PrimitiveShapeRenderer> primitive_renderer) {
        primitive_renderer->GetMaterial()->SetParameter("baseColor",
                                                        texture_.get());
        GetNode()->AddComponent<BoxCollider>(Box{
            {0.0f, 0.0f, -0.005}, {0.5f, texture_aspect_ratio_ / 2.0f, 0.01f}});
      });
}

void WorldSpaceEditorUi::UpdateImGuiMousePosition(float3 world_point) {
  ImGuiIO* io = &ImGui::GetCurrentContext()->IO;
  io->MousePos = CalculateImGuiPointFromWorldPoint(world_point);
}

void WorldSpaceEditorUi::UpdateImGuiMouseDown(bool is_left_click_down,
                                              bool is_right_click_down) {
  ImGuiIO* io = &ImGui::GetCurrentContext()->IO;
  io->MouseDown[0] = is_left_click_down;
  io->MouseDown[1] = is_right_click_down;
}

void WorldSpaceEditorUi::OnActiveStatusChanged(bool is_active) {
  if (texture_ && is_active) {
    GetView().GetHost()->TryGetExtension()->ApplyTextureRenderTarget(
        texture_->GetTexture());
  } else {
    GetView().GetHost()->TryGetExtension()->ApplyTextureRenderTarget(nullptr);
  }
}

ImVec2 WorldSpaceEditorUi::CalculateImGuiPointFromWorldPoint(
    float3 world_point) {
  float3 local_hit = GetNode()->LocalFromWorldPoint(world_point);
  // Convert from quad-space to [0,1] space and flip y.
  float2 normalized_hit = {
      local_hit.x + 0.5f,
      (texture_aspect_ratio_ - (local_hit.y + texture_aspect_ratio_ / 2.0f)) /
          texture_aspect_ratio_};
  return {normalized_hit.x * texture_resolution_.x,
          normalized_hit.y * texture_resolution_.y};
}

void WorldSpaceEditorUi::HandleControllerHitEvent(
    ControllerHitEvent controller_hit_event) {
  ControllerHitEvent::Hand hand = controller_hit_event.GetHand();
  // Only one controller can control WorldSpaceEditorUi at once.
  if (active_hand_.has_value() && hand != active_hand_) {
    return;
  }
  ImGuiIO* io = &ImGui::GetCurrentContext()->IO;
  if (controller_hit_event.GetHitNode() != GetNode()) {
    io->MouseDown[0] = false;
    io->MouseDown[1] = false;
    io->MousePos = {-FLT_MAX, -FLT_MAX};
    active_hand_ = std::nullopt;
    return;
  }
  active_hand_ = controller_hit_event.GetHand();

  // Map "select" or "pinch" actions to left click.
  bool is_left_click_down =
      controller_hit_event
          .GetInputActionCurrentState<bool>(
              /*action_name=*/kDefaultSelectActionName)
          .value_or(false) ||
      AlmostEqual(controller_hit_event
                      .GetInputActionCurrentState<float>(
                          /*action_name=*/kDefaultPinchGestureActionName)
                      .value_or(0.0f),
                  1.0f);

  // Map "back" or "menu" or "system" to right click.
  bool is_right_click_down = controller_hit_event
                                 .GetInputActionCurrentState<bool>(
                                     /*action_name=*/kDefaultBackActionName)
                                 .value_or(false) ||
                             controller_hit_event
                                 .GetInputActionCurrentState<bool>(
                                     /*action_name=*/kDefaultSystemActionName)
                                 .value_or(false) ||
                             controller_hit_event
                                 .GetInputActionCurrentState<bool>(
                                     /*action_name=*/kDefaultMenuActionName)
                                 .value_or(false);

  UpdateImGuiMousePosition(controller_hit_event.GetHit()->world_point);
  UpdateImGuiMouseDown(is_left_click_down, is_right_click_down);
}

}  // namespace imp::editor
