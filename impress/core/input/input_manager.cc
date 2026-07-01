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

#include <cstddef>
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
#include "core/input/input_events_pool.h"
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
    PointerEvent::DeviceType device_type, InputEventSource source) {
  std::vector<PointerEvent>& events = pointer_events_[source].events;
  if (!events.empty()) {
    PointerEvent& last = events.back();
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
  events.push_back(pointer_event_processor_.CreatePointerEvent(
      action, ids, points, elapsed_time, device_type));
  return absl::OkStatus();
}

absl::Status InputManager::ProcessKeyboardInput(uint8_t action, Key key,
                                                absl::Duration elapsed_time,
                                                InputEventSource source) {
  // Validates the keyboard event (action).
  if (action <= static_cast<uint8_t>(KeyboardEventType::kNone) &&
      action >= static_cast<uint8_t>(KeyboardEventType::kMax)) {
    IMP_LOG(imp::ERROR) << "Invalid keyboard event/action (" << action
               << ") sent to InputManager.";
    return absl::InvalidArgumentError("Invalid keyboard event/action.");
  }
  std::vector<KeyboardEvent>& events = keyboard_events_[source].events;
  events.push_back(
      KeyboardEvent(static_cast<KeyboardEventType>(action), key, elapsed_time));
  return absl::OkStatus();
}

void InputManager::ProcessTextInput(absl::string_view contents,
                                    InputEventSource source) {
  std::vector<TextInputEvent>& events = text_input_events_[source].events;
  events.push_back(TextInputEvent(contents));
}

absl::Status InputManager::ProcessWheelInput(float2 delta, float2 point,
                                             absl::Duration elapsed_time,
                                             InputEventSource source) {
  std::vector<WheelEvent>& events = wheel_events_[source].events;
  events.push_back(WheelEvent(delta, point, elapsed_time));
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

bool InputManager::HasPointerEvent() { return pointer_events_.HasEvents(); }

std::vector<PointerEvent> InputManager::PopPointerEvents() {
  return pointer_events_.PopMergedEvents();
}

bool InputManager::HasKeyboardEvent() { return keyboard_events_.HasEvents(); }

std::vector<KeyboardEvent> InputManager::PopKeyboardEvents() {
  return keyboard_events_.PopMergedEvents();
}

bool InputManager::HasTextInputEvent() {
  return text_input_events_.HasEvents();
}

std::vector<TextInputEvent> InputManager::PopTextInputEvents() {
  return text_input_events_.PopMergedEvents();
}

bool InputManager::HasWheelEvent() { return wheel_events_.HasEvents(); }

std::vector<WheelEvent> InputManager::PopWheelEvents() {
  return wheel_events_.PopMergedEvents();
}

void InputManager::Update() {
  IMP_TRACE();

  // Input processing pipeline order:
  // 1. Merge/Sanitize: Consolidate events from different sources and resolve
  //    conflicts (e.g., local vs. remote pointer jumps).
  // 2. Intercept/Filter: Allow interceptors to consume or modify events before
  //    they reach the application.
  // 3. Handle/Dispatch: Pass the remaining events to the active InputHandler
  //    for high-level processing and scene dispatch.

  for (std::unique_ptr<InputInterceptor>& interceptor : input_interceptors_) {
    for (size_t i = 0; i < kInputEventSourceCount; ++i) {
      InputEventSource source = static_cast<InputEventSource>(i);
      interceptor->FilterPointerEvents(pointer_events_[source]);
      interceptor->FilterKeyboardEvents(keyboard_events_[source],
                                        text_input_events_[source]);
      interceptor->FilterWheelEvents(wheel_events_[source]);
      interceptor->FilterInputActionEvents(input_action_events_[source]);
    }
  }
  // The default engine input handlers do not consume text input events via
  // PopTextInputEvents(). They must be explicitly cleared here every frame
  // to prevent memory leaks.
  for (size_t i = 0; i < kInputEventSourceCount; ++i) {
    text_input_events_[static_cast<InputEventSource>(i)].events.clear();
  }

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

void InputManager::PushInputActionEvent(InputActionEvent input_action_event,
                                        InputEventSource source) {
  std::vector<InputActionEvent>& events = input_action_events_[source].events;
  events.push_back(input_action_event);
}

bool InputManager::HasInputActionEvent() {
  return input_action_events_.HasEvents();
}

std::vector<InputActionEvent> InputManager::PopInputActionEvents() {
  return input_action_events_.PopMergedEvents();
}

}  // namespace imp
