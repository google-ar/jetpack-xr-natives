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

#include "core/editor/components/camera_rotate.h"

#include "core/common/log.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/editor_plugin.h"
#include "core/editor/events.h"
#include "core/input/input_manager.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/gestures/drag_gesture.h"

namespace imp::editor {

// How much drag the gesture influences the spin speed. This value was chosen to
// match the default value in Blender.
constexpr float kDragSensitivity = 0.4f;
// Maximum pitch rotation angle.
constexpr float kMaximumRotationAngleDegrees = 60.0f;

void CameraRotate::Setup(NodeHandle pivot, float pitch, float yaw) {
  if (!pivot) {
    IMP_LOG(imp::FATAL) << "CameraRotateComponent needs a pivot to work properly";
  }

  pivot_ = pivot;
  pitch_ = pitch;
  yaw_ = yaw;
  pivot_->SetLocalRotation(QuatFromEuler({pitch_, yaw_, 0}));

  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  Dispatcher& editor_dispatcher = editor.GetDispatcher();

  // Camera transformations
  // TODO: Enable arrow keys for drag gesture movement
  auto drag_gesture_event_listener =
      [this](const DragGesture::UpdateEvent& event) mutable {
        if (event.pointer != kMousePointerIdLeft) {
          return;
        }
        float2 current_rotation_delta = event.delta * kDragSensitivity * -1.0f;
        if (invert_y_enabled_) {
          current_rotation_delta.y *= -1.0f;
        }
        yaw_ += current_rotation_delta.x;
        pitch_ += current_rotation_delta.y;
        pitch_ = clamp(pitch_, -kMaximumRotationAngleDegrees,
                       kMaximumRotationAngleDegrees);
        pivot_->SetLocalRotation(QuatFromEuler(float3(pitch_, yaw_, 0.0f)));
      };

  auto invert_camera_event_listener =
      [this](const InvertCameraYEvent& event) mutable {
        invert_y_enabled_ = event.enabled;
      };

  editor_dispatcher.Connect(drag_gesture_event_listener, this);
  editor_dispatcher.Connect(invert_camera_event_listener, this);

  // When the app camera is ignored, the editor camera is always active and can
  // be controlled by the app dispatcher.
  if (editor.GetCameraConfiguration() ==
      EditorPlugin::CameraConfiguration::kEditorCameraOnly) {
    Dispatcher& app_dispatcher = GetView().GetDispatcher();
    app_dispatcher.Connect(drag_gesture_event_listener, this);
    app_dispatcher.Connect(invert_camera_event_listener, this);
  }
}

}  // namespace imp::editor
