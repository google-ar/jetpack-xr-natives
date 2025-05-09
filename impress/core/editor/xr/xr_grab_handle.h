/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_XR_XR_GRAB_HANDLE_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_XR_XR_GRAB_HANDLE_H_

#include <string>

#include "core/actions/controller_events.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/view/framework/render/mesh_renderer.h"
namespace imp::editor {
// XrGrabHandle is a component that allows the attached Node to be
// dragged-and-dropped in a sphere around the main Impress camera. I.e., The
// Node can be positioned at a fixed difference from the Impress camera and will
// face always face said camera. A color-changing grab handle is rendered at the
// center of the XrGrabHandleNode.
class XrGrabHandle : public imp::Component {
 public:
  // distance_from_camera is the distance from the camera the XrGrabHandle.
  // will be rendered.
  // initial_vertical_offset_degrees is the starting angle above the horizon
  // where the XrGrabHandle will be rendered.
  void Setup(float distance_from_camera,
             float initial_horizontal_offset_degrees,
             float initial_vertical_offset_degrees);
  void Update(const imp::FrameTime& frame_time);

 private:
  // TODO Support raycasts derived from a mouse.
  // Tracks grab state by monitoring per-frame ControllerHitEvents.
  void HandleControllerHitEvent(imp::ControllerHitEvent event);
  // Called per-ControllerHitEvent to update the position of the Node with the
  // XrGrabHandle component.
  void UpdateGrabHandleState(imp::ControllerHitEvent event, bool button_down,
                             bool has_changed_since_last_sync);

  enum class XrGrabHandleState {
    kNotHovered,  // Controller ray does not collide with the XrGrabHandle.
    kHovered,  // Controller ray collides with the XrGrabHandle but the "select"
               // controller button is not pressed.
    kGrabbed   // Controller "select" button has been pressed while hovered and
               // Node is now considered grabbed. In this state, XrGrabHandle
               // continuously follows the controller's ray.
  };

  float4 GetGrabHandleMaterialColor(XrGrabHandleState grab_handle_state);

  XrGrabHandleState grab_state_ = XrGrabHandle::XrGrabHandleState::kNotHovered;
  // Store the MeshRenderer to set material params.
  imp::ComponentHandle<imp::MeshRenderer> mesh_renderer_;
  float distance_from_camera_;
  bool is_select_or_pinch_button_activated_;
  imp::float3 direction_vector_;
  absl::optional<ControllerHitEvent::Hand> active_hand_;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_XR_XR_GRAB_HANDLE_H_
