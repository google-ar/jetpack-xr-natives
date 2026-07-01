// Copyright 2026 Google LLC
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

#include "core/editor/widgets/transform_widget_aspect_rotate.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/camera/camera_component.h"
#include "core/editor/command.h"
#include "core/editor/events.proto.imp.h"
#include "core/editor/node_value_command.h"
#include "core/editor/visualizers/transform_gizmo_assets.h"
#include "core/geometry/shapes/box.h"
#include "core/math/almost_equal.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/collision/cylinder_collider.h"
#include "core/view/utils/asset.h"

namespace imp::editor {

namespace {

// The discard threshold parameter name for the rotate aspect material.
constexpr absl::string_view kDiscardThresholdParam = "discardThreshold";

// The rotate gizmo discards pixels in its fragment shader based on this value.
// The discard works by grabbing a direction vector from the local-space vertex
// position and dotting it with the local-space view direction.
// Values are from -1 to 1, and any value below the threshold is discarded.
constexpr float kDiscardThreshold = -0.2f;

// Rotations to align a cylinder collider with the X, Y, and Z axes.
// The cylinder is aligned with the Y-axis by default, so index 1 is identity.
const quatf kRotateAspectColliderRotations[3] = {
    quatf::fromAxisAngle(float3(0.0f, 0.0f, 1.0f), -M_PI / 2.0f),
    kIdentityQuatf,
    quatf::fromAxisAngle(float3(1.0f, 0.0f, 0.0f), M_PI / 2.0f)};

}  // namespace

void TransformWidgetAspectRotate::CreateCollider(Box bounds) {
  if (collider_node_.IsValid()) return;  // Already has a collider.

  const float3 extents = bounds.getMax() - bounds.getMin();
  float height = 0.0f;
  float radius = 0.0f;
  int active_axis = -1;
  if (!IsCameraFacing()) {
    float3 axis = GetAxis();
    for (int i = 0; i < kNumAxes; ++i) {
      if (axis[i] > 0.0f) {
        active_axis = i;
        break;
      }
    }
  }

  if (active_axis == -1) {
    // If no axis is specified (e.g. camera facing), find the thinnest dimension
    // of the bounding box to use as the height of the cylinder.
    active_axis = 0;
    if (extents[1] < extents[0]) active_axis = 1;
    if (extents[2] < extents[active_axis]) active_axis = 2;
  }

  // If this is the x axis, use the y and z extents to determine the radius.
  // If y, use the x and z. If z, use the x and y.
  const int ii = (active_axis + 1) % kNumAxes;
  const int jj = (active_axis + 2) % kNumAxes;
  // Get the radius of the circle formed by the extents.
  radius = std::max(extents[ii], extents[jj]) / 2.0f;
  // Use the size along the axis of rotation as the height of the cylinder.
  height = extents[active_axis];

  // Create a new node to attach the cylinder collider to.
  // We need to do this since we can't rotate the cylinder collider in its
  // component and instead need to rotate the node that holds the collider.
  collider_node_ = GetNode()->CreateChildNode();
  collider_node_->SetLocalRotation(kRotateAspectColliderRotations[active_axis]);

  auto collider = collider_node_->AddComponent<CylinderCollider>();
  const float3 center_in_collider =
      inverse(collider_node_->GetLocalRotation()) * bounds.center;
  const float3 base = center_in_collider - float3(0.0f, height / 2.0f, 0.0f);
  collider->SetCylinder(base, radius, height);

  // Have this new collider node report back up to the transform widget aspect.
  collider->SetHitNode(GetNode());
}

void TransformWidgetAspectRotate::UpdateWidgetTransform() {
  if (!state_.camera_facing) return;
  // We want the node's Y-axis to face the camera.
  // The camera's world rotation aligns its -Z with the view direction.
  // To align the node's +Y with the camera's +Z (back vector), we rotate the
  // camera's rotation by 90 degrees around the X-axis.
  const quatf cam_rot = GetCamera()->GetNode()->GetWorldRotation();
  const quatf align_y =
      quatf::fromAxisAngle(float3(1.0f, 0.0f, 0.0f), M_PI / 2.0f);
  GetNode()->SetWorldRotation(cam_rot * align_y);
}

float3 TransformWidgetAspectRotate::GetAxis() const {
  if (!state_.camera_facing) return state_.axis.Value();

  // For camera-facing rotation, the axis points towards the camera.
  return -GetCamera()->GetNode()->GetWorldForward();
}

const AssetDefinition& TransformWidgetAspectRotate::GetMaterialAsset() const {
  if (IsCameraFacing()) {
    return transform_gizmo_assets::kTransformGizmoMaterialCmat;
  }

  return transform_gizmo_assets::kTransformGizmoRotateMaterialCmat;
}

void TransformWidgetAspectRotate::UpdateAspect(
    const bool commit, std::vector<std::unique_ptr<Command>>& commands) {
  // Get the screen-space position of the centroid of the selected nodes.
  const std::optional<float2> model_clip_position =
      GetCamera()->PixelFromWorldPoint(centroid_start_);

  if (!model_clip_position.has_value()) return;  // Centroid is off-screen.

  // Vec from the widget center to the pointer position when the drag started.
  const float2 start = normalize(pointer_start_ - *model_clip_position);
  // Convert the start vector to an angle about the model center.
  const float angle_start = atan2(start.y, start.x);
  // Vec from the widget center to the current pointer position.
  const float2 current = normalize(pointer_current_ - *model_clip_position);
  // Convert the current vector to an angle about the model center.
  const float angle_current = atan2(current.y, current.x);
  // The difference between these two angles is the angle to rotate to.
  const float angle = angle_start - angle_current;

  // TODO: support local mode.
  // Quaternion representing the rotation around the selected axis.
  const quatf delta_rot = quatf::fromAxisAngle(GetAxis(), angle);

  for (const NodeTransformData& data : initial_node_transform_data_) {
    if (!data.node) continue;

    // If rotating multiple nodes, they rotate around the group's centroid.
    // Calculate the node's position relative to the centroid before rotation.
    const float3 node_offset = data.position_start - centroid_start_;
    // Rotate the offset vector and add it back to the centroid to get the
    // node's new position.
    const float3 new_pos = centroid_start_ + delta_rot * node_offset;
    // Apply the rotation delta to the node's starting rotation.
    const quatf new_rot = delta_rot * data.rotation_start;

    data.node->SetWorldPosition(new_pos);
    data.node->SetWorldRotation(new_rot);

    // If commit is false, we only update the node visually but don't create
    // undo/redo commands. This happens continuously during the drag.
    // Commands are only added to the stack when the drag finishes.
    if (!commit) continue;

    // Check if the new values are different enough from the starting
    // values to warrant creating an undo entry.
    if (!RoughlyEqual(data.position_start, new_pos)) {
      commands.push_back(std::make_unique<NodeValueCommand<float3>>(
          data.node, data.position_start, new_pos,
          [editor = editor_](NodeHandle target, const float3 value) {
            target->SetWorldPosition(value);
            NodeUpdatedEvent event;
            event.target = target;
            event.translation = value;
            editor->GetDispatcher().Send(event);
          }));
    }

    if (!RoughlyEqual(data.rotation_start, new_rot)) {
      commands.push_back(std::make_unique<NodeValueCommand<quatf>>(
          data.node, data.rotation_start, new_rot,
          [editor = editor_](NodeHandle target, const quatf value) {
            target->SetWorldRotation(value);
            NodeUpdatedEvent event;
            event.target = target;
            event.rotation = value;
            editor->GetDispatcher().Send(event);
          }));
    }
  }
}

void TransformWidgetAspectRotate::SetupMaterialParameters() {
  if (IsCameraFacing()) return;
  material_->SetParameter(kDiscardThresholdParam, kDiscardThreshold);
}

}  // namespace imp::editor
