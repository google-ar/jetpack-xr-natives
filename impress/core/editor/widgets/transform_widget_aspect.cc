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

#include "core/editor/widgets/transform_widget_aspect.h"

#include <cmath>
#include <optional>

#include "absl/status/status.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/common/registry.h"
#include "core/editor/command_manager.h"
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
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/gestures/drag_gesture.h"
#include "core/view/framework/gestures/tap_gesture.h"
#include "core/view/utils/frame_time.h"
#include "mediapipe/framework/port/status_macros.h"

constexpr float kAngleThreshold = 0.003f;

namespace imp::editor {

absl::Status TransformWidgetAspect::Setup() {
  editor_ = &GetView().GetRegistry().Get<Editor>()->get();
  command_manager_ = &GetView().GetRegistry().GetOrCreate<CommandManager>();
  MP_RETURN_IF_ERROR(
      IsHandleValid<&TransformWidgetAspectState::transform_widget_mode_control>(
          state_));

  Dispatcher& editor_dispatcher = editor_->GetDispatcher();
  editor_dispatcher.Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        // We only support single selection for the transform widget.
        active_node_ = editor_->GetSingleSelectedNode();
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

  editor_dispatcher.Connect(
      GetNode(),
      [this](const DragGesture::StartEvent& event) mutable {
        if (!IsActive() || !active_node_) {
          return imp::Dispatcher::kAccept;
        }
        position_start_ = active_node_->GetWorldPosition();
        rotation_start_ = active_node_->GetWorldRotation();
        scale_start_ = active_node_->GetLocalScale();
        pointer_start_ = event.position;
        pointer_current_ = pointer_start_;

        std::optional<float2> model_pixel_position =
            GetCamera()->PixelFromWorldPoint(active_node_->GetWorldPosition());
        if (model_pixel_position.has_value()) {
          pointer_start_delta_from_model_center_ =
              pointer_start_ - *model_pixel_position;
        } else {
          pointer_start_delta_from_model_center_ = {0.0001f, 0.0001f};
        }

        std::optional<float3> closest_point_to_axis = ComputeClosestPoint();
        if (!closest_point_to_axis.has_value()) {
          closest_point_to_axis = position_start_ + GetAxis();
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
  return state_.aspect != TRANSFORM_ASPECT_SCALE
             ? state_.axis
             : normalize(active_node_->WorldFromLocalVector(state_.axis));
}

std::optional<float3> TransformWidgetAspect::ComputeClosestPoint() const {
  Ray pointer = GetCamera()->WorldRayFromPixelPoint(
      pointer_current_ - pointer_start_delta_from_model_center_);
  float3 model_position = active_node_->GetWorldPosition();

  float3 direction = normalize(GetAxis());
  std::optional<float> distance_to_move = collision::ClosestPointOnRayToLine(
      Ray(model_position, direction), pointer);
  if (!distance_to_move.has_value()) {
    return model_position;
  }
  return model_position + *distance_to_move * direction;
}

void TransformWidgetAspect::Update(const FrameTime& frame_time) {
  if (!active_node_) {
    return;
  }

  // Scale is in local-space, so set the rotation of the scale group (the three
  // cubes) to match the world rotation of the model. We know the parent of this
  // node is the scale group because the structure of the transform widget model
  // is known/stable.
  if (state_.aspect == TRANSFORM_ASPECT_SCALE) {
    GetNode()->GetParent()->SetWorldRotation(active_node_->GetWorldRotation());
  }

  if (!dragging_) {
    return;
  }

  UpdateAspect();
}

void TransformWidgetAspect::UpdateAspect(bool commit) {
  if (!active_node_) return;

  switch (state_.aspect) {
    case TRANSFORM_ASPECT_TRANSLATE: {
      std::optional<float3> projection = ComputeClosestPoint();
      if (!projection.has_value()) return;

      active_node_->SetWorldPosition(*projection);

      if (commit && !AlmostEqual(position_start_, *projection)) {
        // Make the transform change undoable.
        command_manager_->PerformCommand<NodeValueCommand<float3>>(
            active_node_, position_start_, *projection,
            [this](NodeHandle target, float3 value) {
              target->SetWorldPosition(value);
              // Send an event to notify that the transform has been updated.
              NodeUpdatedEvent event;
              event.target = target;
              event.translation = value;
              editor_->GetDispatcher().Send(event);
            });
      }
    } break;
    case TRANSFORM_ASPECT_ROTATE: {
      std::optional<float2> model_clip_position =
          GetCamera()->PixelFromWorldPoint(active_node_->GetWorldPosition());
      if (!model_clip_position.has_value()) return;

      float2 start = normalize(pointer_start_ - *model_clip_position);
      float angle_start = atan2(start.y, start.x);
      float2 current = normalize(pointer_current_ - *model_clip_position);
      float angle_current = atan2(current.y, current.x);
      float angle = angle_start - angle_current;
      // TODO: support local mode.
      // active_node_->SetWorldRotation(
      //    quatf::fromAxisAngle(
      //        active_node_->WorldFromLocalVector(state_.axis), angle) *
      //            rotation_start_);
      quatf rotation =
          quatf::fromAxisAngle(state_.axis, angle) * rotation_start_;
      active_node_->SetWorldRotation(rotation);

      if (commit && !AlmostEqual(rotation, rotation_start_)) {
        command_manager_->PerformCommand<NodeValueCommand<quatf>>(
            active_node_, rotation_start_, rotation,
            [this](NodeHandle target, quatf value) {
              target->SetWorldRotation(value);

              // Send an event to notify that the transform has been updated.
              NodeUpdatedEvent event;
              event.target = target;
              event.rotation = value;
              editor_->GetDispatcher().Send(event);
            });
      }
    } break;
    case TRANSFORM_ASPECT_SCALE: {
      std::optional<float3> projection = ComputeClosestPoint();
      if (!projection) return;

      // Zero out the scale so we can use LocalFromWorld without feedback.
      active_node_->SetLocalScale(kOne3);

      // Compute the delta of the cursor along the axis from the start.
      // We need to get the axis in local-space so you are scaling the right
      // axis to visually pull the model along the axis of the 3D widget.
      float3 scale_delta = active_node_->LocalFromWorldVector(
          *projection - pointer_start_closest_point_to_axis_);
      // Since this is a delta along the axis, only one component is non-zero.
      float scale_1D = scale_delta.x + scale_delta.y + scale_delta.z;
      // Scale shouldn't change if the pointer hasn't moved, so subtract the
      // axis and then add it back scaled by scale_1D + 1 since scale_1D should
      // be zero.
      float3 scale = scale_start_ - state_.axis + state_.axis * (1 + scale_1D);
      active_node_->SetLocalScale(scale);

      if (commit && !AlmostEqual(scale, scale_start_)) {
        command_manager_->PerformCommand<NodeValueCommand<float3>>(
            active_node_, scale_start_, scale,
            [this](NodeHandle target, float3 value) {
              target->SetLocalScale(value);
              // Send an event to notify that the transform has been updated.
              NodeUpdatedEvent event;
              event.target = target;
              event.scale = value;
              editor_->GetDispatcher().Send(event);
            });
      }
    } break;
  }
}

}  // namespace imp::editor
