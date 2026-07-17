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

#include "core/input/input_manager.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/actions/input_action_event.h"
#include "core/common/trace.h"
#include "core/input/keyboard_event.h"
#include "core/input/pointer_event.h"
#include "core/input/wheel_event.h"
#include "core/math/vec.h"

namespace imp {

InputManager::InputManager(std::unique_ptr<InputHandlerBase> default_handler)
    : pointer_event_processor_(), pointer_events_() {
  PushInputHandler(std::move(default_handler));
}

InputManager::~InputManager() {}

absl::Status InputManager::ProcessPointerInput(
    uint8_t action, const std::vector<Pointer::Id>& ids,
    const std::vector<float2>& points, absl::Duration elapsed_time,
    PointerEvent::DeviceType device_type) {
  if (!pointer_events_.empty()) {
    PointerEvent& last = pointer_events_.back();
    if (last.ElapsedTime() == elapsed_time &&
        last.Type() == PointerEventType::kMove &&
        last.Type() == static_cast<PointerEventType>(action) &&
        last.GetDeviceType() == device_type) {
      // The host OS may batch together touches into OS-specific events; for
      // example on iOS with two fingers, touchesMoved may be called once or
      // twice per frame.
      return pointer_event_processor_.UpdatePointerEvent(last, ids, points);
    }
  }
  pointer_events_.push_back(pointer_event_processor_.CreatePointerEvent(
      action, ids, points, elapsed_time, device_type));
  return absl::OkStatus();
}

absl::Status InputManager::ProcessKeyboardInput(uint8_t action, Key key,
                                                absl::Duration elapsed_time) {
  // Validates the keyboard event (action).
  if (action <= static_cast<uint8_t>(KeyboardEventType::kNone) &&
      action >= static_cast<uint8_t>(KeyboardEventType::kMax)) {
    IMP_LOG(imp::ERROR) << "Invalid keyboard event/action (" << action
               << ") sent to InputManager.";
    return absl::InvalidArgumentError("Invalid keyboard event/action.");
  }
  keyboard_events_.push_back(
      KeyboardEvent(static_cast<KeyboardEventType>(action), key, elapsed_time));
  return absl::OkStatus();
}

void InputManager::ProcessTextInput(absl::string_view contents) {
  text_input_events_.push_back(TextInputEvent(contents));
}

absl::Status InputManager::ProcessWheelInput(float2 delta, float2 point,
                                             absl::Duration elapsed_time) {
  wheel_events_.push_back(WheelEvent(delta, point, elapsed_time));
  return absl::OkStatus();
}

void InputManager::PushInputHandler(
    std::unique_ptr<InputHandlerBase> input_handler) {
  input_handlers_.push_back(std::move(input_handler));
}

void InputManager::PopInputHandler() {
  // Prevents popping the default handler.
  if (input_handlers_.size() > 1) {
    input_handlers_.pop_back();
  }
}

bool InputManager::HasPointerEvent() { return !pointer_events_.empty(); }

std::vector<PointerEvent> InputManager::PopPointerEvents() {
  std::vector<PointerEvent> result;
  std::swap(result, pointer_events_);
  return result;
}

bool InputManager::HasKeyboardEvent() { return !keyboard_events_.empty(); }

std::vector<KeyboardEvent> InputManager::PopKeyboardEvents() {
  std::vector<KeyboardEvent> result;
  std::swap(result, keyboard_events_);
  return result;
}

bool InputManager::HasTextInputEvent() { return !text_input_events_.empty(); }

std::vector<TextInputEvent> InputManager::PopTextInputEvents() {
  std::vector<TextInputEvent> result;
  std::swap(result, text_input_events_);
  return result;
}

bool InputManager::HasWheelEvent() { return !wheel_events_.empty(); }

std::vector<WheelEvent> InputManager::PopWheelEvents() {
  std::vector<WheelEvent> result;
  std::swap(result, wheel_events_);
  return result;
}

void InputManager::Update() {
  IMP_TRACE();

  for (std::unique_ptr<InputInterceptor>& interceptor : input_interceptors_) {
    interceptor->FilterPointerEvents(pointer_events_);
    interceptor->FilterKeyboardEvents(keyboard_events_, text_input_events_);
    interceptor->FilterWheelEvents(wheel_events_);
    interceptor->FilterInputActionEvents(input_action_events_);
  }
  text_input_events_.clear();

  if (!input_handlers_.empty()) {
    // Calls the input handler at the top of the stack.
    // Note: This is a strategy pattern where a 'handling strategy' can be
    // crafted for a specific platform or behavior. The handler implementation
    // on top of the stack receives the low level input events and then wraps
    // them into higher-level events and dispatches those. Popping the handler
    // reverts to the previous handling strategy. The initial (default)
    // strategy, cannot be popped.
    input_handlers_.back()->Update(this);
  }
}

void InputManager::AddInterceptor(
    std::unique_ptr<InputInterceptor> input_interceptor) {
  input_interceptors_.push_back(std::move(input_interceptor));
}

void InputManager::PushInputActionEvent(InputActionEvent input_action_event) {
  input_action_events_.push_back(input_action_event);
}

bool InputManager::HasInputActionEvent() {
  return !input_action_events_.empty();
}

std::vector<InputActionEvent> InputManager::PopInputActionEvents() {
  std::vector<InputActionEvent> result;
  std::swap(result, input_action_events_);
  return result;
}

}  // namespace imp
