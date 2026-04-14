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

#include "core/editor/widgets/transform_widget_aspect.h"

#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/camera/camera_component.h"
#include "core/camera/camera_manager.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/common/registry.h"
#include "core/editor/command.h"
#include "core/editor/command_manager.h"
#include "core/editor/composite_command.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/editor/events.proto.imp.h"
#include "core/editor/node_value_command.h"
#include "core/editor/widgets/transform_widget_aspect_state.proto.imp.h"
#include "core/math/almost_equal.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/scene_handles/scene_handle_status_utils.h"
#include "core/view/framework/gestures/drag_gesture.h"
#include "core/view/framework/gestures/hover_gesture.h"
#include "core/view/framework/gestures/tap_gesture.h"
#include "core/view/utils/frame_time.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::editor {

namespace {

// The hover sizes for each aspect of the transform widget.
// Corresponds to the order of the aspects in the transform widget aspect
// state proto.
constexpr float kGizmoHoverSizes[] = {1.2f, 1.1f, 1.2f};

// Returns true if node is a descendant of any of the nodes in
// potential_ancestors.
bool IsDescendantOfAny(
    const NodeHandle& node,
    const absl::flat_hash_set<NodeHandle>& potential_ancestors) {
  NodeHandle parent = node->GetParent();
  while (parent) {
    if (potential_ancestors.contains(parent)) {
      return true;
    }
    parent = parent->GetParent();
  }
  return false;
}

}  // namespace

absl::Status TransformWidgetAspect::Setup() {
  editor_ = &GetView().GetRegistry().Get<Editor>()->get();
  command_manager_ = &GetView().GetRegistry().GetOrCreate<CommandManager>();
  hovered_ = false;
  MP_RETURN_IF_ERROR(
      IsHandleValid<&TransformWidgetAspectState::transform_widget_mode_control>(
          state_));

  Dispatcher& editor_dispatcher = editor_->GetDispatcher();
  editor_dispatcher.Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        const absl::flat_hash_set<NodeHandle>& selected =
            editor_->GetSelectedNodes();
        selected_nodes_.assign(selected.begin(), selected.end());
      },
      this);

  // Catch the TapEvent to prevent the editor from doing anything else.
  editor_dispatcher.Connect(
      GetNode(),
      [this](const imp::TapGesture::TapEvent& event) mutable {
        state_.transform_widget_mode_control->CycleMode();
        return imp::Dispatcher::kAccept;
      },
      this);

  // Increase the scale of a handle/aspect when it gets hovered.
  editor_dispatcher.Connect(
      GetNode(),
      [this](const imp::HoverGesture::HoverEvent& event) mutable {
        // At the time this was implemented, HoverGesture and DragGesture were
        // coupled such that dragging would prevent HoverGesture::EXIT from
        // firing until the drag was both complete AND the pointer moved.
        // As a result if you finish dragging, the handle will remain scaled up
        // until the pointer moves again. If the Gestures are decoupled, this
        // should be revisited.
        if (event.state == HoverGesture::ENTER && !hovered_) {
          const float hover_scale =
              kGizmoHoverSizes[static_cast<int>(state_.aspect)];
          hovered_ = true;
          scale_before_hover_ = GetNode()->GetLocalScale();
          GetNode()->SetLocalScale(scale_before_hover_ * hover_scale);
        } else if (event.state == HoverGesture::EXIT && hovered_) {
          hovered_ = false;
          GetNode()->SetLocalScale(scale_before_hover_);
        }
        return imp::Dispatcher::kAccept;
      },
      this);

  editor_dispatcher.Connect(
      GetNode(),
      [this](const DragGesture::StartEvent& event) mutable {
        if (!IsActive() || selected_nodes_.empty()) {
          return imp::Dispatcher::kAccept;
        }

        // Compute the centroid of all the selected nodes including children.
        float3 sum_pos = {0.0f, 0.0f, 0.0f};
        int valid_node_count = 0;
        for (const NodeHandle& node : selected_nodes_) {
          if (!node.IsValid()) continue;
          sum_pos += node->GetWorldPosition();
          valid_node_count++;
        }

        // If there are no valid nodes, return.
        if (valid_node_count == 0) return imp::Dispatcher::kAccept;

        // Centroid is the sum of all positions divided by the # of nodes.
        centroid_start_ = sum_pos / static_cast<float>(valid_node_count);

        // We only want to apply transformations to nodes that are not
        // descendants of other selected nodes. e.g. if we scale a parent and
        // its child by 2x the child will have increased in size by 4x.
        std::vector<NodeHandle> nodes_to_process;
        const absl::flat_hash_set<NodeHandle> selected_nodes_set(
            selected_nodes_.begin(), selected_nodes_.end());
        for (const NodeHandle& node : selected_nodes_) {
          if (IsDescendantOfAny(node, selected_nodes_set)) continue;
          nodes_to_process.push_back(node);
        }

        initial_node_transform_data_.clear();
        initial_node_transform_data_.reserve(nodes_to_process.size());
        for (const NodeHandle& node : nodes_to_process) {
          const float3 pos = node->GetWorldPosition();
          initial_node_transform_data_.push_back(
              {node, pos, node->GetWorldRotation(), node->GetLocalScale()});
        }

        pointer_start_ = event.position;
        pointer_current_ = pointer_start_;

        const std::optional<float2> model_pixel_position =
            GetCamera()->PixelFromWorldPoint(centroid_start_);
        if (model_pixel_position.has_value()) {
          pointer_start_delta_from_model_center_ =
              pointer_start_ - *model_pixel_position;
        } else {
          pointer_start_delta_from_model_center_ = {0.0001f, 0.0001f};
        }

        std::optional<float3> closest_point_to_axis = ComputeClosestPoint();
        if (!closest_point_to_axis.has_value()) {
          closest_point_to_axis = centroid_start_ + GetAxis();
        }
        pointer_start_closest_point_to_axis_ = *closest_point_to_axis;

        dragging_ = true;
        return imp::Dispatcher::kAccept;
      },
      this);

  editor_dispatcher.Connect(
      GetNode(),
      [this](const DragGesture::UpdateEvent& event) mutable {
        pointer_current_ = event.position;
        return imp::Dispatcher::kAccept;
      },
      this);

  editor_dispatcher.Connect(
      GetNode(),
      [this](const DragGesture::FinishEvent& event) mutable {
        dragging_ = false;
        UpdateAspect(/*commit = */ true);
        return imp::Dispatcher::kAccept;
      },
      this);

  return absl::OkStatus();
}

ComponentHandle<CameraComponent> TransformWidgetAspect::GetCamera() const {
  ComponentHandle<CameraComponent> editor_camera = editor_->GetCamera();
  return editor_camera->IsActive() ? editor_camera
                                   : GetView().GetCameraManager().GetCamera();
}

float3 TransformWidgetAspect::GetAxis() const {
  if (state_.aspect != TRANSFORM_ASPECT_SCALE) {
    return state_.axis;
  }

  if (selected_nodes_.size() > 1) {
    return state_.axis;
  }

  if (!selected_nodes_.empty() && selected_nodes_[0].IsValid()) {
    return normalize(selected_nodes_[0]->WorldFromLocalVector(state_.axis));
  }

  return state_.axis;
}

std::optional<float3> TransformWidgetAspect::ComputeClosestPoint() const {
  const Ray pointer = GetCamera()->WorldRayFromPixelPoint(
      pointer_current_ - pointer_start_delta_from_model_center_);

  const float3 model_position = centroid_start_;

  const float3 direction = normalize(GetAxis());
  const std::optional<float> distance_to_move =
      collision::ClosestPointOnRayToLine(Ray(model_position, direction),
                                         pointer);
  if (!distance_to_move.has_value()) {
    return model_position;
  }
  return model_position + *distance_to_move * direction;
}

void TransformWidgetAspect::Update(const FrameTime& frame_time) {
  if (selected_nodes_.empty()) return;

  if (state_.aspect == TRANSFORM_ASPECT_SCALE) {
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

  if (!dragging_) {
    return;
  }

  UpdateAspect();
}

void TransformWidgetAspect::UpdateAspect(bool commit) {
  if (selected_nodes_.empty()) return;

  std::vector<std::unique_ptr<Command>> commands;

  switch (state_.aspect) {
    case TRANSFORM_ASPECT_TRANSLATE:
      UpdateTranslate(commit, commands);
      break;
    case TRANSFORM_ASPECT_ROTATE:
      UpdateRotate(commit, commands);
      break;
    case TRANSFORM_ASPECT_SCALE:
      UpdateScale(commit, commands);
      break;
  }

  if (!commit || commands.empty()) return;

  command_manager_->PerformCommand<CompositeCommand>(std::move(commands));
}

void TransformWidgetAspect::UpdateTranslate(
    const bool commit, std::vector<std::unique_ptr<Command>>& commands) {
  // Calculate the closest point on the widget's axis to the user's pointer.
  // This is in world-space.
  const std::optional<float3> projection = ComputeClosestPoint();

  if (!projection.has_value()) return;  // Pointer and axis are parallel.

  // The projection on the axis is the new position of the centroid.
  const float3 new_centroid = *projection;
  // Calculate how much the centroid has moved since the drag started.
  const float3 delta = new_centroid - centroid_start_;

  // Apply the same translation delta to all selected nodes.
  for (const NodeTransformData& data : initial_node_transform_data_) {
    if (!data.node.IsValid()) continue;

    const float3 new_pos = data.position_start + delta;
    data.node->SetWorldPosition(new_pos);

    // If commit is false, we only update the node visually but don't create
    // undo/redo commands. This happens continuously during the drag.
    // Commands are only added to the stack when the drag finishes and the
    // position has changed enough to matter.
    if (!commit || AlmostEqual(data.position_start, new_pos)) continue;

    commands.push_back(std::make_unique<NodeValueCommand<float3>>(
        data.node, data.position_start, new_pos,
        [this](NodeHandle target, const float3 value) {
          target->SetWorldPosition(value);
          NodeUpdatedEvent event;
          event.target = target;
          event.translation = value;
          editor_->GetDispatcher().Send(event);
        }));
  }
}

void TransformWidgetAspect::UpdateRotate(
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
  const quatf delta_rot = quatf::fromAxisAngle(state_.axis, angle);

  for (const NodeTransformData& data : initial_node_transform_data_) {
    if (!data.node.IsValid()) continue;

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
    if (!AlmostEqual(data.position_start, new_pos)) {
      commands.push_back(std::make_unique<NodeValueCommand<float3>>(
          data.node, data.position_start, new_pos,
          [this](NodeHandle target, const float3 value) {
            target->SetWorldPosition(value);
            NodeUpdatedEvent event;
            event.target = target;
            event.translation = value;
            editor_->GetDispatcher().Send(event);
          }));
    }

    if (!AlmostEqual(data.rotation_start, new_rot)) {
      commands.push_back(std::make_unique<NodeValueCommand<quatf>>(
          data.node, data.rotation_start, new_rot,
          [this](NodeHandle target, const quatf value) {
            target->SetWorldRotation(value);
            NodeUpdatedEvent event;
            event.target = target;
            event.rotation = value;
            editor_->GetDispatcher().Send(event);
          }));
    }
  }
}

void TransformWidgetAspect::UpdateScale(
    const bool commit, std::vector<std::unique_ptr<Command>>& commands) {
  // Calculate the closest point on the widget's axis to the user's pointer.
  // This is in world-space.
  const std::optional<float3> projection = ComputeClosestPoint();

  if (!projection) return;  // Pointer and axis are parallel.

  // Difference between the current projected point on the axis and the initial
  // point when the drag started.
  const float3 delta_vec = *projection - pointer_start_closest_point_to_axis_;

  // Project the delta vector onto the axis with dot to get a 1D scale factor.
  // This represents how much the user has dragged along the axis.
  const float scale_1D = dot(delta_vec, GetAxis());

  // Non-uniform scale along this axis as the default.
  float3 s_vec = float3(1.0f, 1.0f, 1.0f) + state_.axis * scale_1D;

  // Scale uniformly if multiple nodes are selected to prevent skew/shear.
  if (selected_nodes_.size() > 1) {
    s_vec = float3(1.0f + scale_1D);
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
    if (!AlmostEqual(data.position_start, new_pos)) {
      commands.push_back(std::make_unique<NodeValueCommand<float3>>(
          data.node, data.position_start, new_pos,
          [this](NodeHandle target, const float3 value) {
            target->SetWorldPosition(value);
            NodeUpdatedEvent event;
            event.target = target;
            event.translation = value;
            editor_->GetDispatcher().Send(event);
          }));
    }

    if (!AlmostEqual(data.scale_start, new_scale)) {
      commands.push_back(std::make_unique<NodeValueCommand<float3>>(
          data.node, data.scale_start, new_scale,
          [this](NodeHandle target, const float3 value) {
            target->SetLocalScale(value);
            NodeUpdatedEvent event;
            event.target = target;
            event.scale = value;
            editor_->GetDispatcher().Send(event);
          }));
    }
  }
}

}  // namespace imp::editor
