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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_WORLD_SPACE_EDITOR_UI_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_WORLD_SPACE_EDITOR_UI_H_

#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "dear_imgui/imgui.h"
#include "core/actions/controller_events.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/node_handle.h"
#include "core/render/texture.h"
#include "core/view/framework/assets/material_asset.h"
namespace imp::editor {
/*
 * When enabled, WorldSpaceEditorUi renders the Editor on a quad in world space.
 * Tapping on the quad is interpreted as a primary pointer click.
 *
 * Only one WorldSpaceEditorUi can exist in the scene at once.
 *
 * TODO Support multiple panels on different quads/nodes.
 */
class WorldSpaceEditorUi : public Component {
 public:
  // A fatal error will be thrown if a WorldSpaceEditorUi is already in the
  // scene.
  Future<absl::Status> Setup(float2 texture_resolution = {800, 600});

  // Enabling will switch to rendering on a world-space quad.
  // Disabling will restore rendering to screen-space.
  // TODO Use tabbed layout when enabled and switch to the old
  // layout when disabled.
  void OnActiveStatusChanged(bool is_active);

  // Given the components of a ray cast, performs a collision test with the
  // WorldSpaceEditorUi. Updates the ImGui mouse position if a hit occurs.
  void ProcessRayCast(float3 ray_origin, float3 ray_direction);

 private:
  // TODO Support raycasts derived from a mouse.
  // Converts a ControllerHitEvent into simulated mouse input in the Editor UI.
  void HandleControllerHitEvent(ControllerHitEvent controller_hit_event);

  // Given the world space coordinate of a hit on the surface of the
  // WorldSpaceEditorUi, transform it into ImGui pixel coordinates and update
  // the mouse position in the ImGui IO system.
  void UpdateImGuiMousePosition(float3 world_point);

  void UpdateImGuiMouseDown(bool is_left_click_down, bool is_right_click_down);

  // Receives PointerHitEvents and transforms them into ImGui
  // mouse moves and clicks, respectively.
  void ConnectToPointerHitEvents(Dispatcher& dispatcher);

  // Transforms a world point into a pixel coordinate for the ImGui UI.
  ImVec2 CalculateImGuiPointFromWorldPoint(float3 world_point);

  TexturePtr texture_;
  float2 texture_resolution_;
  float texture_aspect_ratio_;
  absl::optional<ControllerHitEvent::Hand> active_hand_;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_EDITOR_QUAD_H_
