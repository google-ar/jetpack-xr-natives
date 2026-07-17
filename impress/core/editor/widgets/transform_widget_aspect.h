/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_ASPECT_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_ASPECT_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/editor/command.h"
#include "core/editor/command_manager.h"
#include "core/editor/editor.h"
#include "core/editor/widgets/transform_widget_aspect_state.proto.imp.h"
#include "core/geometry/shapes/box.h"
#include "core/materials/material.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/gestures/drag_gesture.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

// A component for each arm of the 3D transform widget. Attach this component
// to one of the arm sub-nodes and mark it in the ISF with the appropriate axis
// and TransformAspect (translate, rotate, or scale).
class TransformWidgetAspect : public imp::Component {
 public:
  static constexpr bool kExcludeFromEditor = true;

  imp::Future<absl::Status> Setup();
  void Update(const FrameTime& frame_time);

 private:
  // Creates a box collider for the aspect based on the mesh bounds.
  void CreateBoxCollider(Box bounds);

  // Creates a cylinder collider for the aspect based on the mesh bounds.
  void CreateCylinderCollider(const Box& bounds);

  // Called when a drag gesture starts on the aspect.
  imp::Dispatcher::PropagationResult OnDragStart(
      const DragGesture::StartEvent& event);

  // Called when a drag gesture updates on the aspect.
  imp::Dispatcher::PropagationResult OnDragUpdate(
      const DragGesture::UpdateEvent& event);

  // Called when a drag gesture finishes on the aspect.
  imp::Dispatcher::PropagationResult OnDragFinish(
      const DragGesture::FinishEvent& event);

  // Position, rotation, and scale packet associated with a node.
  struct NodeTransformData {
    NodeHandle node;
    float3 position_start;
    quatf rotation_start;
    float3 scale_start;
  };

  // Helper to get the camera to use (editor or normal).
  ComponentHandle<CameraComponent> GetCamera() const;

  float3 GetAxis() const;

  // Compute the aspect change based on user input. Add undo/redo if commit.
  void UpdateAspect(bool commit = false);

  // Updates the material color based on the current state (hovered, dragging).
  void UpdateMaterial();

  // Computes the closest point between the ray projected by the cursor and the
  // axis this widget controls. (World-space)
  std::optional<float3> ComputeClosestPoint() const;

  // Translation logic for the transform widget.
  void UpdateTranslate(bool commit,
                       std::vector<std::unique_ptr<Command>>& commands);

  // Rotation logic for the transform widget.
  void UpdateRotate(bool commit,
                    std::vector<std::unique_ptr<Command>>& commands);

  // Scale logic for the transform widget.
  void UpdateScale(bool commit,
                   std::vector<std::unique_ptr<Command>>& commands);

  TransformWidgetAspectState state_;

  // The nodes that are currently selected and being transformed.
  std::vector<NodeHandle> selected_nodes_;

  // The command manager to use for undo/redo.
  CommandManager* command_manager_;

  // The pointer position when a drag starts. (Screen-space)
  float2 pointer_start_;

  // The current pointer position. (Screen-space)
  float2 pointer_current_;

  // The delta of the pointer start position and model center. (Screen-space)
  float2 pointer_start_delta_from_model_center_;

  // The closest point on this widget's axis to the pointer when a drag
  // starts. (World-space)
  float3 pointer_start_closest_point_to_axis_;

  // The centroid of the selected nodes when a drag starts. (World-space)
  float3 centroid_start_;

  // The initial transform data of the selected nodes when a drag starts.
  // (World-space)
  std::vector<NodeTransformData> initial_node_transform_data_;

  // Whether the user is currently dragging the widget.
  bool dragging_;
  Editor* editor_;
  // Whether the transform widget handle/aspect is being hovered.
  bool hovered_;

  // The material instance used for the aspect.
  OwnedMaterialPtr material_;

 public:
  using IsfInfo =
      IsfInfo<&TransformWidgetAspect::state_, IsfDependencies<GltfRenderer>>;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_ASPECT_H_
