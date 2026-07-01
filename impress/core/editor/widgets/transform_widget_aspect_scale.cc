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

#include "core/editor/widgets/transform_widget_aspect_scale.h"

#include <memory>
#include <optional>
#include <vector>

#include "core/camera/camera_component.h"
#include "core/editor/command.h"
#include "core/editor/events.proto.imp.h"
#include "core/editor/node_value_command.h"
#include "core/math/almost_equal.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

void TransformWidgetAspectScale::UpdateWidgetTransform() {
  if (IsDragging()) return;  // Don't flip orientation while dragging.

  // If we're in scale mode with only a single node selected, align the
  // transform widget to the node's world rotation.
  if (selected_nodes_.size() == 1 && selected_nodes_[0].IsValid()) {
    GetNode()->GetParent()->SetWorldRotation(
        selected_nodes_[0]->GetWorldRotation());
  } else {
    // If multiple nodes are selected, use an identity rotation.
    GetNode()->GetParent()->SetWorldRotation(kIdentityQuatf);
  }
}

float3 TransformWidgetAspectScale::GetAxis() const {
  if (selected_nodes_.size() > 1) {
    return state_.axis.Value();
  }

  if (!selected_nodes_.empty() && selected_nodes_[0].IsValid()) {
    // If dragging, use the initial rotation of the node to compute the axis.
    // Otherwise, use the current world rotation.
    if (IsDragging() && !initial_node_transform_data_.empty()) {
      return initial_node_transform_data_[0].rotation_start *
             state_.axis.Value();
    }
    return selected_nodes_[0]->GetWorldRotation() * state_.axis.Value();
  }

  return state_.axis.Value();
}

void TransformWidgetAspectScale::UpdateAspect(
    const bool commit, std::vector<std::unique_ptr<Command>>& commands) {
  // Calculate the closest point on the widget's axis to the user's pointer.
  // This is in world-space.
  const std::optional<float3> projection = ComputeClosestPoint();

  if (!projection) return;  // Pointer and axis are parallel.

  // Difference between the current projected point on the axis and the initial
  // point when the drag started.
  const float3 delta_vec = *projection - pointer_start_closest_point_to_axis_;

  int active_axes = 0;
  if (state_.axis.Value().x > 0.0f) active_axes++;
  if (state_.axis.Value().y > 0.0f) active_axes++;
  if (state_.axis.Value().z > 0.0f) active_axes++;

  float3 s_vec;
  if (active_axes >= 2) {
    float3 initial_dir = pointer_start_closest_point_to_axis_ - centroid_start_;
    float initial_dist = length(initial_dir);

    // If clicked very close to the center (e.g. uniform scale cube), use camera
    // up and right as the positive drag direction for scaling up.
    if (initial_dist < 0.05f) {
      float3 cam_right = GetCamera()->GetNode()->WorldFromLocalVector(
          float3(1.0f, 0.0f, 0.0f));
      float3 cam_up = GetCamera()->GetNode()->WorldFromLocalVector(
          float3(0.0f, 1.0f, 0.0f));
      initial_dir = normalize(cam_right + cam_up);
    } else {
      initial_dir /= initial_dist;
    }

    const float scale_1D = dot(delta_vec, initial_dir);

    if (active_axes == 3 || selected_nodes_.size() > 1) {
      s_vec = float3(1.0f + scale_1D);
    } else {
      s_vec = float3(1.0f) + state_.axis.Value() * scale_1D;
    }
  } else {
    // Project the delta vector onto the axis with dot to get a 1D scale factor.
    // This represents how much the user has dragged along the axis.
    const float scale_1D = dot(delta_vec, GetAxis());

    // Non-uniform scale along this axis as the default.
    s_vec = float3(1.0f, 1.0f, 1.0f) + state_.axis.Value() * scale_1D;

    // Scale uniformly if multiple nodes are selected to prevent skew/shear.
    if (selected_nodes_.size() > 1) {
      s_vec = float3(1.0f + scale_1D);
    }
  }

  for (const NodeTransformData& data : initial_node_transform_data_) {
    if (!data.node.IsValid()) continue;

    // When scaling multiple objects we also need to translate them towards/away
    // from the centroid to get a true group scale.
    float3 offset = data.position_start - centroid_start_;
    float3 new_pos = centroid_start_ + s_vec * offset;

    // Apply the scale factor to the node's starting scale.
    float3 new_scale = data.scale_start * s_vec;

    data.node->SetWorldPosition(new_pos);
    data.node->SetLocalScale(new_scale);

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

    if (!RoughlyEqual(data.scale_start, new_scale)) {
      commands.push_back(std::make_unique<NodeValueCommand<float3>>(
          data.node, data.scale_start, new_scale,
          [editor = editor_](NodeHandle target, const float3 value) {
            target->SetLocalScale(value);
            NodeUpdatedEvent event;
            event.target = target;
            event.scale = value;
            editor->GetDispatcher().Send(event);
          }));
    }
  }
}

}  // namespace imp::editor
