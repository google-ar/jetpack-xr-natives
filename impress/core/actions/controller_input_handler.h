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

#ifndef THIRD_PARTY_IMPRESS_CORE_ACTIONS_CONTROLLER_INPUT_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_ACTIONS_CONTROLLER_INPUT_HANDLER_H_
#include <string>

#include "absl/strings/string_view.h"
#include "core/actions/controller_events.h"
#include "core/actions/input_action_event.h"
#include "core/input/input_manager.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"

namespace imp {

// ControllerInputHandler processes the queue of InputActionEvents from the
// InputManager and sends resulting ControllerHitEvents to the Dispatcher. One
// ControllerHitEvent is sent per set, per subaction path, per Update loop.
//
// In order to identify which action corresponds to a controller's
// transform (there may be many Transform<float> actions),
// ControllerInputHandler looks for the specified raycast_action_name
//
// Similarly, left_hand_subaction_path and right_hand_subaction_path are used as
// default identifiers for left and right controller subaction paths.
//
// TODO: Send lossy PointerHitEvents from ControllerInputHandler.
// Send a hybrid event containing a std::variant<PointerHitEvent,
// ControllerHitEvent>
// TODO: Support DoubleRayHit in ControllerInputHandler by sharing
// a base class with PointerHitEvent.
// TODO Investigate further generalizing ControllerInputHandler.
class ControllerInputHandler : public InputHandlerBase {
 public:
  explicit ControllerInputHandler(BaseView* view,
                                  absl::string_view raycast_action_name,
                                  absl::string_view left_hand_subaction_path,
                                  absl::string_view right_hand_subaction_path)
      : view_(view),
        raycast_action_name_(raycast_action_name),
        left_controller_subaction_path_(left_hand_subaction_path),
        right_controller_subaction_path_(right_hand_subaction_path) {}

  // Pops all InputActionEvents from the InputManager.
  void Update(InputManager* input_manager) override;

  // Processes an individual InputActionEvent.
  // First, this function tries to produce a ControllerHitEvent.
  // If a controller cannot be identified, InputActionStateChangedEvents are
  // sent instead using SendInputActionStateChangedEvents.
  void ProcessInputActionEvent(InputActionEvent event);

  // For each active and updated InputActionState in a given InputActionEvent,
  // sends an InputActionStateChangedEvent.
  void SendInputActionStateChangedEvents(InputActionEvent event);

  // For each active and updated InputActionState, sends an
  // InputActionStateChangedEvent.
  template <typename T>
  void SendInputActionStateChangedEvents(
      absl::string_view action_set_name, absl::string_view subaction_path,
      StringMap<InputActionState<T>> action_name_to_input_action_state) {
    for (const auto& [action_name, input_action_state] :
         action_name_to_input_action_state) {
      SendInputActionStateChangedEvent<T>(action_set_name, subaction_path,
                                          input_action_state);
    }
  }

  // If the given InputActionState is active and updated, send an
  // InputActionStateChangedEvent.
  template <typename T>
  void SendInputActionStateChangedEvent(
      absl::string_view action_set_name, absl::string_view subaction_path,
      InputActionState<T> input_action_state) {
    if (!input_action_state.is_active ||
        !input_action_state.has_changed_since_last_sync) {
      return;
    }
    view_->GetDispatcher().Send(InputActionStateChangedEvent<T>(
        action_set_name, subaction_path, input_action_state));
  }

 private:
  BaseView* view_;
  // Used to locate the InputActionEvent<Transform<float>> to use for the
  // controller's ray cast.
  std::string raycast_action_name_;
  // Used to identify the left controller.
  std::string left_controller_subaction_path_;
  // Used to identify the right controller.
  std::string right_controller_subaction_path_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ACTIONS_CONTROLLER_INPUT_HANDLER_H_
