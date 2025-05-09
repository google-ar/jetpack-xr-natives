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

#include "core/view/framework/input/desktop_gesture_emulator.h"

#include <memory>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "core/view/framework/gestures/double_tap_gesture.h"

namespace imp {
namespace {

// A helper method that combines key and event data into unique id.
uint32_t GetActivationKey(KeyboardEventType event, VirtualKeyCode vk_code,
                          Flags<KeyModifier> modifiers) {
  return static_cast<uint32_t>(event) ^ static_cast<uint32_t>(vk_code) ^
         static_cast<uint32_t>(modifiers.RawValue());
}

// A helper method for instantiating a desktop gesture emulator of type T.
template <typename T>
auto CreateGestureType(BaseView* view, KeyboardEventType key_state,
                       VirtualKeyCode key, Flags<KeyModifier> modifiers) {
  return std::make_pair(GetActivationKey(key_state, key, modifiers),
                        std::make_unique<T>(view, key_state, key, modifiers));
}

// Produces a drag event when activated
class DesktopDragGesture : public DesktopGestureEmulator::BaseGesture {
 public:
  DesktopDragGesture(BaseView* view, KeyboardEventType key_state,
                     VirtualKeyCode key, Flags<KeyModifier> modifiers)
      : BaseGesture(view, key_state, key, modifiers) {}
  void UpdateGesture(const DesktopCombinedInputEvent& input,
                     uint32_t activation_key) override {
    if (input.pointer_events.empty()) return;

    for (const PointerEvent& pointer_event : input.pointer_events) {
      if (pointer_event.Type() == PointerEventType::kDown) {
        input_active_ = true;
        pointer_handler_.DispatchHitEvents(
            PointerEvent(PointerEventType::kUp, {pointer_event.GetPointer()}, 1,
                         pointer_event.ElapsedTime()));
        pointer_handler_.DispatchHitEvents(pointer_event);
      } else if (input_active_ &&
                 pointer_event.Type() == PointerEventType::kMove) {
        pointer_handler_.DispatchHitEvents(pointer_event);
      } else if (input_active_ &&
                 pointer_event.Type() == PointerEventType::kUp) {
        pointer_handler_.DispatchHitEvents(pointer_event);
        input_active_ = false;
      } else {
        input_active_ = false;
      }
    }
  }
};

// Produces a double tap gesture event when activated
class DesktopDoubleTap : public DesktopGestureEmulator::BaseGesture {
 public:
  DesktopDoubleTap(BaseView* view, KeyboardEventType key_state,
                   VirtualKeyCode key, Flags<KeyModifier> modifiers)
      : BaseGesture(view, key_state, key, modifiers), view_(view) {}
  void UpdateGesture(const DesktopCombinedInputEvent& /*input*/,
                     uint32_t /*activation_key*/) override {
    view_->GetDispatcher().Send(
        imp::NodeHandle(),
        DoubleTapGesture::TapEvent(0, PointerEventType::kUp, imp::NodeHandle(),
                                   float2(0, 0)));
  }

 private:
  BaseView* view_;
};

// Produces a pinch gesture event when activated
class DesktopPinchGesture : public DesktopGestureEmulator::BaseGesture {
 public:
  DesktopPinchGesture(BaseView* view, KeyboardEventType key_state,
                      VirtualKeyCode key, Flags<KeyModifier> modifiers)
      : BaseGesture(view, key_state, key, modifiers) {
    scale_pointers_.resize(2);
  }
  void UpdateGesture(const DesktopCombinedInputEvent& input,
                     uint32_t activation_key) override {
    constexpr int kDefaultMousePointerId = 0;
    if (TestModifiers(activation_key)) {
      // Scale amount is from [-kMaxScaleAmount, kMaxScaleAmount]
      const float kMaxScaleAmount = 500.0f;
      // Scales the raw wheel delta values.
      const float kWheelAmplitude = 2.0f;
      // Returns early if not a wheel event.
      if (!input.wheel_event) {
        return;
      }
      // Establishes the 'finger' starting positions.
      const float2 first_point = float2(0.0f, 0.0f);
      const float2 second_point =
          first_point + float2(kMaxScaleAmount * 4.0f, 0.0f);

      // Scales the wheel delta.
      float wheel_delta = kWheelAmplitude * input.wheel_event->GetDelta();

      // Clamps wheel accumulation (and therefore scale) between upper and
      // lower maximums.
      if (wheel_accumulation_ + wheel_delta >= -kMaxScaleAmount &&
          wheel_accumulation_ + wheel_delta <= kMaxScaleAmount) {
        wheel_accumulation_ += wheel_delta;
      }
      float2 wheel_position(wheel_accumulation_, 0.0f);

      // Conjures the pointers needed for scaling.
      scale_pointers_[0] = {
          kDefaultMousePointerId,
          first_point - wheel_position,
          input.wheel_event->GetDelta(),
      };
      scale_pointers_[1] = {
          kDefaultMousePointerId + 1,
          second_point + wheel_position,
          input.wheel_event->GetDelta(),
      };

      // Updates the scaling gesture.
      if (!GestureActive()) {
        // Clears an ongoing scaling event.
        pointer_handler_.DispatchHitEvents(
            PointerEvent(PointerEventType::kUp, scale_pointers_, 2,
                         input.wheel_event->GetElapsedTime()));
        // Starts a new scaling event.
        pointer_handler_.DispatchHitEvents(
            PointerEvent(PointerEventType::kDown, scale_pointers_, 2,
                         input.wheel_event->GetElapsedTime()));
        SetGestureActive();
      } else {
        // Updates scale event.
        pointer_handler_.DispatchHitEvents(
            PointerEvent(PointerEventType::kMove, scale_pointers_, 2,
                         input.wheel_event->GetElapsedTime()));
      }
    } else if (GestureActive()) {
      PointerEvent scale_event(PointerEventType::kUp, scale_pointers_, 2,
                               input.keyboard_events.back().elapsed_time);
      pointer_handler_.DispatchHitEvents(scale_event);
      SetGestureInActive();
      wheel_accumulation_ = 0;
    }
  }

 private:
  float wheel_accumulation_ = 0.0f;
  std::vector<Pointer> scale_pointers_;
};

}  // namespace

DesktopGestureEmulator::BaseGesture::BaseGesture(BaseView* view,
                                                 KeyboardEventType key_state,
                                                 VirtualKeyCode key,
                                                 Flags<KeyModifier> modifiers)
    : pointer_handler_(view),
      key_state_(key_state),
      key_(key),
      modifiers_(modifiers),
      activation_key_(GetActivationKey(key_state, key, modifiers)) {}

DesktopGestureEmulator::DesktopGestureEmulator(BaseView* view) : view_(view) {
  Resume();
}

void DesktopGestureEmulator::Resume() {
  pointer_event_connection_ = view_->GetDispatcher().Connect(
      [this](const DesktopCombinedInputEvent& input) mutable {
        if (!input.keyboard_events.empty()) {
          // Updates the current key.
          const KeyboardEvent& keyboard_event = input.keyboard_events.back();
          current_activation_key_ =
              GetActivationKey(keyboard_event.type, keyboard_event.key.code,
                               keyboard_event.key.modifiers);
        }
        if (!active_gesture_) {
          // Look for gesture to activate.
          if (auto iter = gesture_activators_.find(current_activation_key_);
              iter != gesture_activators_.end()) {
            active_gesture_ = iter.value().get();
          }
        }
        if (active_gesture_ &&
            (active_gesture_->TestModifiers(current_activation_key_) ||
             active_gesture_->GestureActive())) {
          // Updates the active gesture.
          active_gesture_->UpdateGesture(input, current_activation_key_);
        } else if (active_gesture_ && !active_gesture_->GestureActive()) {
          // Clears the active gesture.
          active_gesture_ = nullptr;
        }
      },
      view_);
}

void DesktopGestureEmulator::Pause() { pointer_event_connection_.Disconnect(); }

uint32_t DesktopGestureEmulator::SetGestureActivationKey(
    GestureType gesture, KeyboardEventType key_state, VirtualKeyCode key,
    KeyModifier modifier) {
  return SetGestureActivationKey(gesture, key_state, key,
                                 ToFlags<KeyModifier>(modifier));
}

uint32_t DesktopGestureEmulator::SetGestureActivationKey(
    GestureType gesture, KeyboardEventType key_state, VirtualKeyCode key,
    Flags<KeyModifier> modifiers) {
  switch (gesture) {
    case GestureType::kDrag:
      gesture_activators_.insert(CreateGestureType<DesktopDragGesture>(
          view_, key_state, key, modifiers));
      break;
    case GestureType::kPinch:
      gesture_activators_.insert(CreateGestureType<DesktopPinchGesture>(
          view_, key_state, key, modifiers));
      break;
    case GestureType::kDoubleTap:
      gesture_activators_.insert(CreateGestureType<DesktopDoubleTap>(
          view_, key_state, key, modifiers));
      break;
    default:
      IMP_LOG(imp::ERROR) << "Gesture type activation not implemented.";
      break;
  }
  return GetActivationKey(key_state, key, modifiers);
}

void DesktopGestureEmulator::RemoveGestureActivationKey(
    uint32_t activation_key) {
  gesture_activators_[activation_key] = {};
}

}  // namespace imp
