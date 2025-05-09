/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_ARCORE_AR_IMP_CORE_VIEW_FRAMEWORK_INPUT_DESKTOP_GESTURE_EMULATOR
#define THIRD_PARTY_ARCORE_AR_IMP_CORE_VIEW_FRAMEWORK_INPUT_DESKTOP_GESTURE_EMULATOR

#include <memory>

#include "core/common/enum_flags.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/base_view.h"
#include "core/view/framework/input/desktop_input_handler.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp {
// Converts pointer input into gesture events when gesture is activated through
// user specified activation input keys. Allows for matching gestures to
// activation keys, hold the activation keys paired with pointer input then
// generates gesture events.
class DesktopGestureEmulator {
 public:
  // Lists the supported gesture emulations.
  enum class GestureType {
    kDrag,
    kPinch,
    kDoubleTap,
  };

  class BaseGesture {
   public:
    BaseGesture(BaseView* view, KeyboardEventType key_state, VirtualKeyCode key,
                Flags<KeyModifier> modifiers);
    virtual ~BaseGesture() = default;
    virtual void UpdateGesture(const DesktopCombinedInputEvent& input,
                               uint32_t activation_key) = 0;
    virtual bool GestureActive() { return input_active_; }
    virtual bool TestModifiers(uint32_t activation_key) {
      return activation_key_ == activation_key;
    }

   protected:
    void SetGestureActive() { input_active_ = true; }
    void SetGestureInActive() { input_active_ = false; }

    PointerInputHandler pointer_handler_;
    KeyboardEventType key_state_;
    VirtualKeyCode key_;
    Flags<KeyModifier> modifiers_;
    uint32_t activation_key_ = 0;
    bool input_active_ = false;
  };

  explicit DesktopGestureEmulator(BaseView* view);
  // Resumes all input handling.
  void Resume();
  // Pauses all input handling.
  void Pause();
  // Sets the gesture to activate and the keyboard input required to activate
  // it.
  uint32_t SetGestureActivationKey(GestureType gesture,
                                   KeyboardEventType key_state,
                                   VirtualKeyCode key, KeyModifier modifier);
  // Sets the gesture to activate and the keyboard input required to activate
  // it.
  uint32_t SetGestureActivationKey(GestureType gesture,
                                   KeyboardEventType key_state,
                                   VirtualKeyCode key,
                                   Flags<KeyModifier> modifiers);
  // Removes a gesture activation scheme.
  void RemoveGestureActivationKey(uint32_t activation_key);

 private:
  BaseView* view_;
  Dispatcher::Connection pointer_event_connection_;
  BaseGesture* active_gesture_ = nullptr;
  RobinMap<uint32_t, std::unique_ptr<BaseGesture>> gesture_activators_;
  uint32_t current_activation_key_ = 0;
};
}  // namespace imp

#endif  // THIRD_PARTY_ARCORE_AR_IMP_CORE_VIEW_FRAMEWORK_INPUT_DESKTOP_GESTURE_EMULATOR
