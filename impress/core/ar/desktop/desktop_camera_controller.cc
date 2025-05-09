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

#include "core/ar/desktop/desktop_camera_controller.h"

#include <memory>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "core/ar/desktop/ar_desktop_data.h"
#include "core/input/key_codes.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/input/desktop_input_handler.h"

namespace imp {
namespace ar {
// Default FOV for the 3D viewer that is close to the typical FOV of an AR Core
// Camera.
constexpr float kDefaultFovDegrees = 64.0f;
constexpr static float kDefaultVerticalFit = 2.0f / 3.0f;
constexpr static float kNearPlane = 0.01f;
constexpr static float kFarPlane = 30.0f;
constexpr static float kYawStart = 15.0f;
constexpr static float kPitchStart = -45.0f;

constexpr float3 kCameraStartPosition(0.1f, 0.75f, 0.1f);

std::unique_ptr<DesktopInputHandler> CreateInputHandler(BaseView* view) {
  return std::make_unique<DesktopInputHandler>(view);
}

DesktopCameraController::DesktopCameraController(BaseView* view)
    : view_(view),
      last_pointer_(0.0f),
      pointer_(0.0f),
      position_(kCameraStartPosition),
      move_vector_(kZero3),
      active_(false),
      desktop_gesture_emulator_(view) {
  desktop_gesture_emulator_.SetGestureActivationKey(
      imp::DesktopGestureEmulator::GestureType::kDrag,
      imp::KeyboardEventType::kOnDown, imp::VirtualKeyCode::VK_NONE,
      ToFlags(imp::KeyModifier::LCTRL));
  desktop_gesture_emulator_.SetGestureActivationKey(
      imp::DesktopGestureEmulator::GestureType::kDrag,
      imp::KeyboardEventType::kOnDown, imp::VirtualKeyCode::VK_NONE,
      ToFlags(imp::KeyModifier::RCTRL));
  desktop_gesture_emulator_.SetGestureActivationKey(
      imp::DesktopGestureEmulator::GestureType::kPinch,
      imp::KeyboardEventType::kOnDown, imp::VirtualKeyCode::VK_NONE,
      ToFlags(imp::KeyModifier::LCTRL, imp::KeyModifier::LSHIFT));
  desktop_gesture_emulator_.SetGestureActivationKey(
      imp::DesktopGestureEmulator::GestureType::kPinch,
      imp::KeyboardEventType::kOnDown, imp::VirtualKeyCode::VK_NONE,
      ToFlags(imp::KeyModifier::RCTRL, imp::KeyModifier::RSHIFT));
  desktop_gesture_emulator_.SetGestureActivationKey(
      imp::DesktopGestureEmulator::GestureType::kDoubleTap,
      imp::KeyboardEventType::kOnUp, imp::VirtualKeyCode::VK_t,
      imp::KeyModifier::NONE);
}

void DesktopCameraController::Update() {
  const float kScale = 0.25f;
  const float kTimeScale = 6.25f;
  float2 delta = kScale * (pointer_ - last_pointer_);
  last_pointer_ = pointer_;
  if (dot(move_vector_, move_vector_) > kFltEpsilon) {
    move_vector_ = normalize(move_vector_);
  }

  // Accumulates rotation.
  yaw_ *= quatf::fromAxisAngle(kUp, ToRadians(-delta.x));
  pitch_ *= quatf::fromAxisAngle(kRight, ToRadians(-delta.y));

  orientation_ = normalize(
      lerp(orientation_, yaw_ * pitch_,
           absl::ToDoubleSeconds(view_->GetFrameTime().GetDeltaTime()) *
               kTimeScale));

  constexpr float kMoveScale = 0.01f;
  position_ += kMoveScale * (orientation_ * move_vector_);
}

void DesktopCameraController::Initialize(bool load_virtual_environment) {
  if (load_virtual_environment) {
    LoadDesktopEnvironment();
  }

  // Configure camera.
  position_ = kCameraStartPosition;
  yaw_ = quatf::fromAxisAngle(kUp, ToRadians(kYawStart));
  pitch_ = quatf::fromAxisAngle(kRight, ToRadians(kPitchStart));
  orientation_ = yaw_ * pitch_;
  projection_matrix_ = mat4::perspective(
      kDefaultFovDegrees, kDefaultVerticalFit, kNearPlane, kFarPlane);
}

void DesktopCameraController::Resume() {
  assert(Executor::CurrentExecutor() == Executor::ForegroundExecutor());
  if (desktop_environment_) {
    desktop_environment_->SetEnabled(true);
  }
  view_->GetInputManager().PushInputHandler(CreateInputHandler(view_));
  desktop_gesture_emulator_.Resume();

  // Connect to desktop input event.
  bool input_started = false;
  input_event_connection_ = view_->GetDispatcher().Connect(
      [this, input_started](const DesktopCombinedInputEvent& input) mutable {
        static Flags<KeyModifier> modifiers = ToFlags(KeyModifier::NONE);
        if (!input.keyboard_events.empty()) {
          modifiers = input.keyboard_events.back().key.modifiers;
        }
        if (input.pointer_events.empty()) {
          return;
        }

        if (modifiers == ToFlags(KeyModifier::NONE)) {
          for (auto& pointer_event : input.pointer_events) {
            if (pointer_event.Type() == PointerEventType::kDown) {
              input_started = true;
              this->last_pointer_ = this->pointer_ =
                  pointer_event.GetPointer().point;
            }
            if (input_started &&
                pointer_event.Type() == PointerEventType::kMove) {
              this->pointer_ = pointer_event.GetPointer().point;
            }
            if (pointer_event.Type() == PointerEventType::kUp) {
              input_started = false;
            }
          }
        } else {
          input_started = false;
        }
      },
      view_);
  // Listens for WASD keys to move the camera.
  keyboard_event_connection_ = view_->GetDispatcher().Connect(
      [this](const DesktopCombinedInputEvent& input) mutable {
        constexpr float kStep = 1.0f;
        if (input.keyboard_events.empty()) {
          return;
        }
        const KeyboardEvent& keyboard_event = input.keyboard_events.back();
        auto code = keyboard_event.key.code;
        if (keyboard_event.type == KeyboardEventType::kOnDown &&
            code == VirtualKeyCode::VK_w) {
          move_vector_.z = -kStep;
        } else if (keyboard_event.type == KeyboardEventType::kOnDown &&
                   code == VirtualKeyCode::VK_s) {
          move_vector_.z = kStep;
        } else {
          move_vector_.z = 0.0f;
        }

        if (keyboard_event.type == KeyboardEventType::kOnDown &&
            code == VirtualKeyCode::VK_a) {
          move_vector_.x = -kStep;
        } else if (keyboard_event.type == KeyboardEventType::kOnDown &&
                   code == VirtualKeyCode::VK_d) {
          move_vector_.x = kStep;
        } else {
          move_vector_.x = 0.0f;
        }
      },
      view_);

  if (desktop_environment_) {
    desktop_environment_->SetEnabled(true);
  }

  active_ = true;
}

void DesktopCameraController::Pause() {
  if (desktop_environment_) {
    desktop_environment_->SetEnabled(true);
  }
  view_->GetInputManager().PopInputHandler();

  desktop_gesture_emulator_.Pause();
  input_event_connection_.Disconnect();
  keyboard_event_connection_.Disconnect();

  if (desktop_environment_) {
    desktop_environment_->SetEnabled(false);
  }
  active_ = false;
}

void DesktopCameraController::Shutdown() {
  assert(Executor::CurrentExecutor() == Executor::ForegroundExecutor());
  if (desktop_environment_) {
    view_->DestroyNode(desktop_environment_);
  }
}

mat4f DesktopCameraController::GetViewMatrix() {
  mat4f view(orientation_);
  view[3].xyz = position_;
  return view;
}

void DesktopCameraController::LoadDesktopEnvironment() {
  assert(Executor::CurrentExecutor() == Executor::ForegroundExecutor());
  if (desktop_environment_) {
    return;
  }
  desktop_environment_ = view_->CreateNode();
  pending_model_ =
      desktop_environment_
          ->AddComponent<GltfRenderer>(ar_desktop_data::kApartmentGlb)
          .Then([this](const absl::StatusOr<ComponentHandle<GltfRenderer>>&
                           statusor) -> Future<absl::Status> {
            if (!statusor.ok()) {
              IMP_LOG(imp::ERROR) << statusor.status();
            } else {
              desktop_environment_->GetComponent<imp::BoxCollider>()->SetMask(
                  CollisionMask::kNone);
              desktop_environment_->SetEnabled(this->active_);
            }

            return Future<absl::Status>(statusor.status());
          });
}

bool DesktopCameraController::IsReadyToRender() const {
  if (!desktop_environment_) return true;
  return pending_model_.Ready() && pending_model_.Get().ok();
}

}  // namespace ar
}  // namespace imp
