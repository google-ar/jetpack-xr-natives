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
#include <optional>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/actions/action_config.h"
#include "core/actions/controller_events.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/editor/components/spatial_ui_canvas.h"
#include "core/editor/editor.h"
#include "core/editor/xr/xr_grab_handle.h"
#include "core/input/pointer_event.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/utils/string_map.h"
#include "core/window/filament_host.h"

namespace imp::editor {

Future<absl::Status> WorldSpaceEditorUi::Setup(
    float2 texture_resolution, StringMap<float3> canvas_position_map) {
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

  canvas_position_map_ = std::move(canvas_position_map);

  // Listen for ControllerHitEvents on the Editor dispatcher
  Dispatcher& editor_dispatcher =
      GetView().GetRegistry().Get<editor::Editor>()->get().GetDispatcher();
  editor_dispatcher.Connect(
      [this](const ControllerHitEvent& event) mutable {
        HandleControllerHitEvent(event);
      },
      this);
  // Listen to PointerHitEvents.
  ConnectToPointerHitEvents(editor_dispatcher);
  ConnectToPointerHitEvents(GetView().GetDispatcher());

  GetNode()->SetName(kWorldUiName);
  return Future<absl::Status>(absl::OkStatus());
}

void WorldSpaceEditorUi::Cleanup() {
  for (auto it = spatial_ui_canvases_.begin(); it != spatial_ui_canvases_.end();
       ++it) {
    GetView().DestroyNode(it->second);
  }
  spatial_ui_canvases_.clear();
}

Future<absl::Status> WorldSpaceEditorUi::UpdateSpatialUiCanvas(
    SpatialUiCanvas::SpatialUiCanvasSettings settings) {
  auto it = spatial_ui_canvases_.find(settings.name);
  if (it != spatial_ui_canvases_.end()) {
    ComponentHandle<SpatialUiCanvas> canvas =
        it->second->GetComponent<SpatialUiCanvas>();
    if (canvas) {
      // If the canvas already exists, update it if the settings have changed.
      // Skip if the canvas is still being created.
      if (canvas->GetSettings().content_position != settings.content_position ||
          canvas->GetSettings().content_size != settings.content_size) {
        canvas->UpdateCanvas(settings);
      }
    }
    return Future<absl::Status>(absl::OkStatus());
  }

  NodeHandle canvas_node = GetNode()->GetView().CreateNode();
  spatial_ui_canvases_[settings.name] = canvas_node;
  return canvas_node
      ->AddComponent<SpatialUiCanvas>(
          /*name=*/settings.name,
          /*content_position=*/settings.content_position,
          /*content_size=*/settings.content_size, texture_.Borrow())
      .Then([this, canvas_node, settings](
                absl::StatusOr<ComponentHandle<SpatialUiCanvas>> canvas) {
        if (!canvas.ok()) {
          IMP_LOG(imp::ERROR) << "Failed to create SpatialUiCanvas: " << canvas.status();
          spatial_ui_canvases_.erase(canvas_node->GetName());
          GetNode()->GetView().DestroyNode(canvas_node);
          return canvas.status();
        }

        NodeHandle grab_handle = GetNode()->GetView().CreateNode();
        grab_handle->SetParent(GetNode());
        grab_handle->SetName(absl::StrCat("grab_handle_", settings.name));
        float3 panel_position = GetKnownCanvasPosition(settings.name);
        grab_handle->AddComponent<XrGrabHandle>(
            /*distance_from_camera=*/panel_position.x,
            /*horizontal_offset=*/panel_position.y,
            /*vertical_offset=*/panel_position.z);

        canvas_node->SetName(settings.name);
        canvas_node->SetParent(grab_handle);

        return absl::OkStatus();
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
  return hit_canvas_->CalculateImGuiPointFromWorldPoint(world_point);
}

void WorldSpaceEditorUi::HandleControllerHitEvent(
    ControllerHitEvent controller_hit_event) {
  ControllerHitEvent::Hand hand = controller_hit_event.GetHand();
  // Only one controller can control WorldSpaceEditorUi at once.
  if (active_hand_.has_value() && hand != active_hand_) {
    return;
  }

  NodeHandle hit_node = controller_hit_event.GetHitNode();
  if (!hit_node) {
    return;
  }

  hit_canvas_ = ComponentHandle<SpatialUiCanvas>();
  auto it = spatial_ui_canvases_.find(hit_node->GetName());
  if (it != spatial_ui_canvases_.end() && it->second == hit_node) {
    hit_canvas_ = it->second->GetComponent<SpatialUiCanvas>();
  }

  ImGuiIO* io = &ImGui::GetCurrentContext()->IO;
  if (!hit_canvas_.IsValid()) {
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

void WorldSpaceEditorUi::ConnectToPointerHitEvents(Dispatcher& dispatcher) {
  // Simulate mouse move events when a raycast hits the quad.
  Dispatcher::Connection connection = dispatcher.Connect(
      [this](const PointerHitEvent& pointer_hit_event) {
        if (pointer_hit_event.GetPointerCount() == 0 || !IsActive()) {
          return;
        }

        NodeHandle hit_node = pointer_hit_event.GetHitNode();
        if (!hit_node) {
          return;
        }

        hit_canvas_ = ComponentHandle<SpatialUiCanvas>();
        auto it = spatial_ui_canvases_.find(hit_node->GetName());
        if (it != spatial_ui_canvases_.end() && it->second == hit_node) {
          hit_canvas_ = it->second->GetComponent<SpatialUiCanvas>();
        }
        if (!hit_canvas_) {
          UpdateImGuiMouseDown(false, false);
          return;
        }
        PointerEvent pointer_event = pointer_hit_event.event;
        switch (pointer_event.Type()) {
          case PointerEventType::kMove:
          case PointerEventType::kHover: {
            UpdateImGuiMousePosition(
                pointer_hit_event.GetTruncatedRayHit()->world_point);
            break;
          }
          case imp::PointerEventType::kDown: {
            UpdateImGuiMouseDown(true, false);
            break;
          }
          case imp::PointerEventType::kUp: {
            UpdateImGuiMouseDown(false, false);
            break;
          }
          default:
            break;
        }
      },
      this);
}

float3 WorldSpaceEditorUi::GetKnownCanvasPosition(
    absl::string_view canvas_name) const {
  auto it = canvas_position_map_.find(canvas_name);
  if (it != canvas_position_map_.end()) {
    return it->second;
  }
  return WorldSpaceEditorUi::kDefaultCanvasPosition;
}

}  // namespace imp::editor
