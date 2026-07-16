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

#include "core/editor/xr/xr_grab_handle.h"

#include <optional>
#include <utility>

#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/actions/action_config.h"
#include "core/actions/controller_events.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/camera/camera_manager.h"
#include "core/collision/ray.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/xr/xr_grab_handle_assets.h"
#include "core/input/pointer_event.h"
#include "core/materials/material.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_factory.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/render/mesh_renderer.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/input/split_engine_input_event.h"

namespace imp::editor {
namespace {
constexpr absl::string_view kMainColorParameterName = "mainColor";
// Dark blue
constexpr float4 kNotHoveredColor = {0.14f, 0.29f, 0.51f, 1.0f};
// Light blue
constexpr float4 kHoveredColor = {0.19f, 0.41f, 0.69f, 1.0f};
// Lighter blue
constexpr float4 kGrabbedColor = {0.2f, 0.9f, 0.9f, 1.0f};

XrGrabHandle::InputSourceType GetInputSourceType(
    const ControllerHitEvent::Hand& hand) {
  if (hand == ControllerHitEvent::Hand::kLeft) {
    return XrGrabHandle::InputSourceType::kControllerLeftHand;
  }
  return XrGrabHandle::InputSourceType::kControllerRightHand;
}

XrGrabHandle::InputSourceType GetInputSourceType(
    const android_xr::SplitEngineInputEvent::PointerType& pointer_type) {
  if (pointer_type == android_xr::SplitEngineInputEvent::PointerType::LEFT) {
    return XrGrabHandle::InputSourceType::kSplitEngineLeftHand;
  }
  return XrGrabHandle::InputSourceType::kSplitEngineRightHand;
}
}  // namespace

constexpr float kGrabHandleSize = 0.1f;

void XrGrabHandle::Setup(float distance_from_camera,
                         float initial_horizontal_offset_degrees,
                         float initial_vertical_offset_degrees) {
  distance_from_camera_ = distance_from_camera;
  // Create a quad to act as a grab handle.
  mesh_renderer_ = GetNode()->AddComponent<MeshRenderer>();
  mesh_renderer_->SetMesh(
      GetView().GetMeshFactory().CreateRegularPolygon(10, kGrabHandleSize));
  direction_vector_ = QuatFromEuler({initial_vertical_offset_degrees,
                                     -initial_horizontal_offset_degrees, 0}) *
                      kForward;
  GetView()
      .GetAssetManager()
      .LoadMaterial(::editor::xr_grab_handle_assets::kSolidMaterialCmat)
      .Then([this](AssetPtr<MaterialAsset> material_asset) {
        // Add a box collider to the quad to catch raycasts.
        GetNode()->AddComponent<BoxCollider>(
            Box{{0.0f, 0.0f, 0.005f},
                float3{kGrabHandleSize, kGrabHandleSize, 0.01f}});
        MaterialPtr material =
            GetView().GetMaterialFactory().CreateMaterial(material_asset);
        mesh_renderer_->SetMaterial(std::move(material));
        mesh_renderer_->GetMaterial()->SetParameter(
            kMainColorParameterName,
            GetGrabHandleMaterialColor(XrGrabHandleState::kNotHovered));
      })
      .Then([this] {
        // Listen for ControllerHitEvents on the Editor dispatcher
        Dispatcher& editor_dispatcher = GetView()
                                            .GetRegistry()
                                            .Get<editor::Editor>()
                                            ->get()
                                            .GetDispatcher();
        editor_dispatcher.Connect(
            [this](const ControllerHitEvent& event) mutable {
              HandleControllerHitEvent(event);
            },
            this);
        editor_dispatcher.Connect(
            [this](const PointerHitEvent& event) mutable {
              HandlePointerHitEvent(event);
            },
            this);
        GetView().GetDispatcher().Connect(
            [this](const android_xr::SplitEngineInputEvent& event) mutable {
              HandleSplitEngineInputEvent(event);
            },
            this);
      })
      .KeptBy(this);
}

void XrGrabHandle::Update(const FrameTime& frame_time) {
  float3 camera_position =
      GetView().GetCameraManager().GetCamera()->GetNode()->GetWorldPosition();
  // Position the node.
  GetNode()->SetWorldPosition(camera_position +
                              (direction_vector_ * distance_from_camera_));
  // Face the camera.
  GetNode()->SetLocalForward(direction_vector_, kUp);
}

void XrGrabHandle::HandleControllerHitEvent(ControllerHitEvent event) {
  if (active_input_source_.has_value() &&
      GetInputSourceType(event.GetHand()) != active_input_source_) {
    return;
  }

  bool is_select_or_pinch_action_activated =
      event
          .GetInputActionCurrentState<bool>(
              /*action_name=*/kDefaultSelectActionName)
          .value_or(false) ||
      AlmostEqual(event
                      .GetInputActionCurrentState<float>(
                          /*action_name=*/kDefaultPinchGestureActionName)
                      .value_or(0.0f),
                  1.0f);

  bool has_changed = is_select_or_pinch_action_activated !=
                     is_select_or_pinch_button_activated_;

  absl::optional<float3> hit_world_point;
  if (event.GetHitNode() == GetNode()) {
    hit_world_point = event.GetHit()->world_point;
  }
  UpdateGrabHandleState(hit_world_point, GetInputSourceType(event.GetHand()),
                        event.GetControllerRay(),
                        /*button_down=*/is_select_or_pinch_action_activated,
                        /*has_changed=*/has_changed);
}

void XrGrabHandle::UpdateGrabHandleState(
    absl::optional<float3> hit_world_point,
    XrGrabHandle::InputSourceType input_source_type, Ray ray, bool button_down,
    bool has_changed) {
  // Logic to transition between states.
  if (hit_world_point.has_value()) {
    active_input_source_ = input_source_type;
    // If select button was pressed down this frame, start grabbing.
    if (button_down && has_changed) {
      grab_state_ = XrGrabHandle::XrGrabHandleState::kGrabbed;
    } else if (!button_down) {
      grab_state_ = XrGrabHandle::XrGrabHandleState::kHovered;
    }
  } else if (active_input_source_.has_value() &&
             input_source_type == active_input_source_) {
    if (grab_state_ == XrGrabHandle::XrGrabHandleState::kGrabbed) {
      // If the component is in a grabbed state but there is no hit, we still
      // want to move the panel to the end of the controller ray.
      hit_world_point = ray.origin + (ray.direction * distance_from_camera_);
    } else {
      grab_state_ = XrGrabHandle::XrGrabHandleState::kNotHovered;
      active_input_source_ = std::nullopt;
    }
  }

  // Set grab handle color.
  mesh_renderer_->GetMaterial()->SetParameter(
      kMainColorParameterName, GetGrabHandleMaterialColor(grab_state_));

  if (grab_state_ != XrGrabHandle::XrGrabHandleState::kGrabbed) {
    return;
  }
  direction_vector_ = normalize(
      *hit_world_point -
      GetView().GetCameraManager().GetCamera()->GetNode()->GetWorldPosition());
}

void XrGrabHandle::HandleSplitEngineInputEvent(
    const android_xr::SplitEngineInputEvent& event) {
  if (active_input_source_.has_value() &&
      GetInputSourceType(event.pointer_type) != active_input_source_) {
    return;
  }

  bool is_action_activated = (event.button_state != 0);

  bool has_changed = is_action_activated != is_split_engine_button_activated_;
  is_split_engine_button_activated_ = is_action_activated;

  absl::optional<float3> hit_world_point;
  if (event.hit_node && event.hit_node->target == GetNode()) {
    hit_world_point = event.hit_node->world_hit_position.value_or(
        event.hit_node->hit_position);
  }
  UpdateGrabHandleState(
      hit_world_point, GetInputSourceType(event.pointer_type),
      Ray{event.origin, normalize(event.direction - event.origin)},
      /*button_down=*/is_action_activated, has_changed);
}

void XrGrabHandle::HandlePointerHitEvent(imp::PointerHitEvent event) {
  if (event.GetHitNode() == GetNode()) {
    PointerEvent pointer_event = event.event;
    switch (pointer_event.Type()) {
      case PointerEventType::kMove:
      case PointerEventType::kHover: {
        if (grab_state_ == XrGrabHandle::XrGrabHandleState::kNotHovered) {
          grab_state_ = XrGrabHandle::XrGrabHandleState::kHovered;
        }
        break;
      }
      case imp::PointerEventType::kDown: {
        grab_state_ = XrGrabHandle::XrGrabHandleState::kGrabbed;
        break;
      }
      case imp::PointerEventType::kUp: {
        grab_state_ = XrGrabHandle::XrGrabHandleState::kHovered;
        break;
      }
      default:
        break;
    }
    if (grab_state_ == XrGrabHandle::XrGrabHandleState::kGrabbed) {
      direction_vector_ = normalize(event.GetTruncatedRayHit()->world_point -
                                    GetView()
                                        .GetCameraManager()
                                        .GetCamera()
                                        ->GetNode()
                                        ->GetWorldPosition());
    }
  } else {
    grab_state_ = XrGrabHandle::XrGrabHandleState::kNotHovered;
  }
}

float4 XrGrabHandle::GetGrabHandleMaterialColor(
    editor::XrGrabHandle::XrGrabHandleState grab_handle_state) {
  switch (grab_handle_state) {
    case XrGrabHandle::XrGrabHandleState::kNotHovered: {
      return kNotHoveredColor;
    }
    case XrGrabHandle::XrGrabHandleState::kHovered: {
      return kHoveredColor;
    }
    case XrGrabHandle::XrGrabHandleState::kGrabbed: {
      return kGrabbedColor;
    }
  }
}

}  // namespace imp::editor
