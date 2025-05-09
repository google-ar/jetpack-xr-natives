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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_INPUT_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_INPUT_MANAGER_H_

#include <memory>
#include <queue>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/actions/input_action_event.h"
#include "core/collision/collision_flags.h"
#include "core/common/optional_error.h"
#include "core/input/keyboard_event.h"
#include "core/input/pointer_event.h"
#include "core/input/pointer_event_processor.h"
#include "core/input/textinput_event.h"
#include "core/input/wheel_event.h"
#include "core/math/transform.h"
#include "core/math/vec.h"

namespace imp {

// LINT.IfChange(pointer_event_api)

// These match the javascript MouseEvent.button values (ex: right = 2).
static constexpr uint32_t kDefaultMousePointerId = 0;
static constexpr uint32_t kMousePointerIdLeft = kDefaultMousePointerId;
static constexpr uint32_t kMousePointerIdRight = 2;

// LINT.ThenChange(
//     //depot/google3/third_party/impress/javascript/core/scripting/api/input_forwarder.ts:pointer_event_api
// )

class InputManager;

struct InputHandlerBase {
  virtual ~InputHandlerBase() {}
  virtual void Update(InputManager* input_manager) = 0;
};

// Dev mode installs an instance of a type to prevent tap events for imgui
// widgets from making it up to the view.
struct InputInterceptor {
  virtual ~InputInterceptor() {}
  virtual void FilterPointerEvents(
      std::vector<PointerEvent>& pointer_events) = 0;
  virtual void FilterKeyboardEvents(
      std::vector<KeyboardEvent>& keyboard_events,
      std::vector<TextInputEvent>& text_input_events) = 0;
  virtual bool TryConsumeWheelEvent(const WheelEvent& wheel_event) = 0;
  virtual void FilterInputActionEvents(
      std::vector<InputActionEvent>& input_action_events) = 0;
};

// The InputManager handles receiving input and queues events to send on the
// next frame.
class InputManager {
 public:
  explicit InputManager(std::unique_ptr<InputHandlerBase> default_handler);
  ~InputManager();

  // Takes/adds in a pointer action, id, position, and elapsed time and add it
  // to the PointerEvent queue.
  absl::Status ProcessPointerInput(uint8_t action,
                                   const std::vector<Pointer::Id>& ids,
                                   const std::vector<float2>& points,
                                   absl::Duration elapsed_time);
  // Takes/adds in a keyboard action, key, and elapsed time and add it to the
  // KeyboardEvent queue.
  absl::Status ProcessKeyboardInput(uint8_t action, Key key,
                                    absl::Duration elapsed_time);
  // Take/adds in a text input contents and add it toTextInputEvent queue.
  void ProcessTextInput(absl::string_view contents);
  // Takes/adds in a scroll wheel delta and elapsed time and add it to the
  // stored WheelEvent.
  absl::Status ProcessWheelInput(float delta, absl::Duration elapsed_time);

  // Pushes an InputActionEvent to the end of the queue.
  void PushInputActionEvent(InputActionEvent input_action_event);

  // Returns true if any InputActionEvents are on the InputActionEvent vector.
  bool HasInputActionEvent();

  // Returns any InputActionEvents queued since the last call, and clears the
  // queue.
  std::vector<InputActionEvent> PopInputActionEvents();

  // Pushes an input handler on top of the input handling stack. The handler on
  // the top of the stack receives the input messages and handles the
  // dispatching of events.
  void PushInputHandler(std::unique_ptr<InputHandlerBase> input_handler);
  // Pops the top most handler of the input stack.
  void PopInputHandler();

  // Returns true if there is at least one queued PointerEvent.
  bool HasPointerEvent();
  // Returns any events queued since the last request, and clears the queue.
  std::vector<PointerEvent> PopPointerEvents();

  // Returns true if there is at least one queued KeyboardEvent.
  bool HasKeyboardEvent();
  // Returns any events queued since the last request, and clears the queue.
  std::vector<KeyboardEvent> PopKeyboardEvents();

  // Returns true if there is at least one queued TextInputEvent.
  bool HasTextInputEvent();
  // Returns any events queued since the last request, and clears the queue.
  std::vector<TextInputEvent> PopTextInputEvents();

  // Returns true if there is a stored wheel event.
  bool HasWheelEvent();
  // Removes the currently stored wheel event.
  WheelEvent PopWheelEvent();

  // Update the current input handler.
  void Update();

  void AddInterceptor(std::unique_ptr<InputInterceptor> input_interceptor);
  absl::Status PopInterceptor();

 private:
  PointerEventProcessor pointer_event_processor_;
  std::vector<std::unique_ptr<InputHandlerBase>> input_handlers_;
  std::vector<PointerEvent> pointer_events_;
  std::vector<KeyboardEvent> keyboard_events_;
  std::vector<TextInputEvent> text_input_events_;
  absl::optional<WheelEvent> wheel_event_;
  std::vector<std::unique_ptr<InputInterceptor>> input_interceptors_;
  std::vector<InputActionEvent> input_action_events_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_INPUT_MANAGER_H_
