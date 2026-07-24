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

#include "core/editor/widgets/transform_widget_aspect_translate.h"

#include <memory>
#include <optional>
#include <vector>

#include "core/editor/command.h"
#include "core/editor/events.proto.imp.h"
#include "core/editor/node_value_command.h"
#include "core/math/almost_equal.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

void TransformWidgetAspectTranslate::UpdateWidgetTransform() {
  // If using world space or translating multiple nodes, use the world axis.
  if (!use_local_space_ || !HasOneNodeSelected()) {
    GetNode()->GetParent()->SetWorldRotation(kIdentityQuatf);
    return;
  }

  // If using world space or translating multiple nodes, use the world axis.
  GetNode()->GetParent()->SetWorldRotation(
      selected_nodes_[0]->GetWorldRotation());
}

float3 TransformWidgetAspectTranslate::GetAxis() const {
  // If using world space or rotating multiple nodes, use the world axis.
  if (!use_local_space_ || !HasOneNodeSelected()) {
    return GetAxisFromState();
  }

  // If dragging, base the axis on the initial rotation of the node.
  if (IsDragging() && !initial_node_transform_data_.empty()) {
    return initial_node_transform_data_[0].rotation_start * GetAxisFromState();
  }

  // Otherwise, base it on the node's current world rotation.
  return selected_nodes_[0]->GetWorldRotation() * GetAxisFromState();
}

float3 TransformWidgetAspectTranslate::GetAxisFromState() const {
  return state_.axis.Value();
}

void TransformWidgetAspectTranslate::UpdateAspect(
    const bool commit, std::vector<std::unique_ptr<Command>>& commands) {
  // Calculate the closest point on the widget's axis to the user's pointer.
  // This is in world-space.
  const std::optional<float3> projection = ComputeClosestPoint();

  if (!projection.has_value()) return;  // Pointer and axis are parallel.

  // The projection on the axis is the new position of the centroid.
  const float3 new_centroid = *projection;
  // Calculate how much the centroid has moved since the drag started,
  // projected onto the active axes/plane.
  float3 delta;
  const float3 world_delta = new_centroid - centroid_start_;
  int active_axes = 0;
  const float3 axis = GetAxisFromState();
  if (axis.x > 0.0f) active_axes++;
  if (axis.y > 0.0f) active_axes++;
  if (axis.z > 0.0f) active_axes++;

  switch (active_axes) {
    case 1:
      delta = GetSingleAxisDelta(world_delta);
      break;
    case 2:
      delta = GetDualAxisDelta(world_delta, axis);
      break;
    default:
      delta = world_delta;
      break;
  }

  // Apply the same translation delta to all selected nodes.
  for (const NodeTransformData& data : initial_node_transform_data_) {
    if (!data.node.IsValid()) continue;

    const float3 new_pos = data.position_start + delta;
    data.node->SetWorldPosition(new_pos);

    // If commit is false, we only update the node visually but don't create
    // undo/redo commands. This happens continuously during the drag.
    // Commands are only added to the stack when the drag finishes and the
    // position has changed enough to matter.
    if (!commit || RoughlyEqual(data.position_start, new_pos)) continue;

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
}

float3 TransformWidgetAspectTranslate::GetSingleAxisDelta(
    const float3& world_delta) const {
  const float3 direction = normalize(GetAxis());
  return dot(world_delta, direction) * direction;
}

float3 TransformWidgetAspectTranslate::GetDualAxisDelta(
    const float3& world_delta, const float3& axis) const {
  float3 plane_normal = kOne3 - axis;

  if (use_local_space_ && HasOneNodeSelected()) {
    quatf rotation;

    // If dragging, use the initial rotation of the node to compute the delta.
    // Otherwise, use the current world rotation.
    if (IsDragging() && !initial_node_transform_data_.empty()) {
      rotation = initial_node_transform_data_[0].rotation_start;
    } else {
      rotation = selected_nodes_[0]->GetWorldRotation();
    }

    plane_normal = rotation * plane_normal;
  }

  return world_delta - dot(world_delta, plane_normal) * plane_normal;
}

}  // namespace imp::editor
