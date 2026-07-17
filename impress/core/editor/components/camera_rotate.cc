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

#include <memory>
#include <vector>

#include "core/common/log.h"
#include "core/actions/input_action_event.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/editor_plugin.h"
#include "core/editor/events.h"
#include "core/input/input_manager.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/input/pointer_event.h"
#include "core/input/wheel_event.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/gestures/drag_gesture.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

namespace {

// How much drag the gesture influences the spin speed. This value was chosen to
// match the default value in Blender.
constexpr float kDragSensitivity = 0.4f;

// Maximum pitch rotation angle.
constexpr float kMaximumRotationAngleDegrees = 60.0f;

// Speed of the free-cam movement.
constexpr float kFreeCamSpeed = 10.0f;

// Speed of the free-cam movement when shift is pressed.
constexpr float kFreeCamShiftSpeed = 30.0f;

}  // namespace

class FreecamInterceptor : public InputInterceptor {
 public:
  explicit FreecamInterceptor(ComponentHandle<CameraRotate> camera_rotate)
      : camera_rotate_(camera_rotate) {}

  void FilterPointerEvents(std::vector<PointerEvent>& pointer_events) override {
    if (camera_rotate_) {
      Editor& editor =
          camera_rotate_->GetView().GetRegistry().Get<Editor>()->get();
      if (editor.UseLegacyCameraControls()) return;

      camera_rotate_->FilterPointerEvents(pointer_events);
    }
  }

  void FilterKeyboardEvents(
      std::vector<KeyboardEvent>& keyboard_events,
      std::vector<TextInputEvent>& text_input_events) override {
    if (camera_rotate_) {
      Editor& editor =
          camera_rotate_->GetView().GetRegistry().Get<Editor>()->get();
      if (editor.UseLegacyCameraControls()) return;

      camera_rotate_->FilterKeyboardEvents(keyboard_events);
    }
  }

  void FilterWheelEvents(std::vector<WheelEvent>& wheel_events) override {}
  void FilterInputActionEvents(
      std::vector<InputActionEvent>& input_action_events) override {}

 private:
  ComponentHandle<CameraRotate> camera_rotate_;
};

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

  GetView().GetInputManager().AddInterceptor(
      std::make_unique<FreecamInterceptor>(
          GetNode()->GetComponent<CameraRotate>()));

  // Camera transformations
  auto drag_gesture_event_listener =
      [this](const DragGesture::UpdateEvent& event) mutable {
        Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
        if (!editor.UseLegacyCameraControls()) return;

        if (event.pointer != kMousePointerIdLeft) return;

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

void CameraRotate::FilterPointerEvents(
    std::vector<PointerEvent>& pointer_events) {
  for (const PointerEvent& event : pointer_events) {
    for (const Pointer& pointer : event.GetChangedPointers()) {
      if (pointer.id != kMousePointerIdRight) continue;

      const PointerEventType event_type = event.Type();
      switch (event_type) {
        case PointerEventType::kDown:
          camera_rotate_enabled_ = true;
          break;
        case PointerEventType::kUp:
        case PointerEventType::kCancel:
          camera_rotate_enabled_ = false;
          break;
        case PointerEventType::kMove:
          if (camera_rotate_enabled_) {
            rotation_delta_ += pointer.delta;
          }
          break;
        default:
          break;
      }
    }
  }
}

void CameraRotate::FilterKeyboardEvents(
    std::vector<KeyboardEvent>& keyboard_events) {
  auto it = keyboard_events.begin();
  while (it != keyboard_events.end()) {
    const bool is_down = it->type == KeyboardEventType::kOnDown;
    bool consume = false;

    switch (it->key.code) {
      case VirtualKeyCode::VK_w:
        w_pressed_ = is_down;
        consume = camera_rotate_enabled_;
        break;
      case VirtualKeyCode::VK_s:
        s_pressed_ = is_down;
        consume = camera_rotate_enabled_;
        break;
      case VirtualKeyCode::VK_a:
        a_pressed_ = is_down;
        consume = camera_rotate_enabled_;
        break;
      case VirtualKeyCode::VK_d:
        d_pressed_ = is_down;
        consume = camera_rotate_enabled_;
        break;
      case VirtualKeyCode::VK_q:
        q_pressed_ = is_down;
        consume = camera_rotate_enabled_;
        break;
      case VirtualKeyCode::VK_e:
        e_pressed_ = is_down;
        consume = camera_rotate_enabled_;
        break;
      case VirtualKeyCode::VK_LEFT_SHIFT:
      case VirtualKeyCode::VK_RIGHT_SHIFT:
        shift_pressed_ = is_down;
        consume = camera_rotate_enabled_;
        break;
      case VirtualKeyCode::VK_r:
        consume = camera_rotate_enabled_;
        break;
      default:
        break;
    }

    if (consume) {
      it = keyboard_events.erase(it);
    } else {
      ++it;
    }
  }
}

void CameraRotate::Update(const FrameTime& frame_time) {
  if (!camera_rotate_enabled_) {
    rotation_delta_ = kZero2;
    return;
  }

  HandleRotation();
  HandleMovement(frame_time);
}

void CameraRotate::HandleRotation() {
  if (length2(rotation_delta_) <= 0.0f) return;

  float2 current_rotation_delta = rotation_delta_ * kDragSensitivity * -1.0f;

  current_rotation_delta.y *= invert_y_enabled_ ? -1.0f : 1.0f;

  yaw_ += current_rotation_delta.x;
  pitch_ += current_rotation_delta.y;
  // Clamp pitch to prevent flipping/gimbal lock.
  pitch_ = clamp(pitch_, -kMaximumRotationAngleDegrees,
                 kMaximumRotationAngleDegrees);
  pivot_->SetLocalRotation(QuatFromEuler(float3(pitch_, yaw_, 0.0f)));
  rotation_delta_ = kZero2;
}

void CameraRotate::HandleMovement(const FrameTime& frame_time) {
  // Use the camera's world rotation to determine movement direction.
  const quatf orientation = GetNode()->GetWorldRotation();
  float3 move_dir = kZero3;

  // Give W priority over S.
  if (w_pressed_)
    move_dir += orientation * kForward;
  else if (s_pressed_)
    move_dir += orientation * kBack;

  // Give A priority over D.
  if (a_pressed_)
    move_dir += orientation * kLeft;
  else if (d_pressed_)
    move_dir += orientation * kRight;

  // Give E priority over Q.
  if (e_pressed_)
    move_dir += orientation * kUp;
  else if (q_pressed_)
    move_dir += orientation * kDown;

  if (length2(move_dir) <= 0.0f) return;

  // Prevent diagonal movement from being faster than cardinal movement.
  move_dir = normalize(move_dir);

  const float speed = shift_pressed_ ? kFreeCamShiftSpeed : kFreeCamSpeed;

  pivot_->SetWorldPosition(pivot_->GetWorldPosition() +
                           move_dir * speed * frame_time.GetDeltaSeconds());
}

}  // namespace imp::editor
