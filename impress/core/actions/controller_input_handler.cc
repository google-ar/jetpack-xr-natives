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

#include "core/actions/controller_input_handler.h"

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "core/actions/controller_events.h"
#include "core/actions/input_action_event.h"
#include "core/collision/ray.h"
#include "core/common/platform_helpers.h"
#include "core/input/input_manager.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/collision/ray_hit.h"
#include "imp.h"

namespace imp {

void ControllerInputHandler::Update(InputManager* input_manager) {
  if (!input_manager->HasInputActionEvent()) {
    return;
  }
  // Pop all InputActionEvents from the InputManager.
  std::vector<InputActionEvent> input_action_events =
      input_manager->PopInputActionEvents();
  for (const InputActionEvent& input_action_event : input_action_events) {
    ProcessInputActionEvent(input_action_event);
  }
}

void ControllerInputHandler::ProcessInputActionEvent(
    InputActionEvent input_action_event) {
  ControllerHitEvent::Hand hand;
  if (input_action_event.subaction_path == left_controller_subaction_path_) {
    hand = ControllerHitEvent::Hand::kLeft;
  } else if (input_action_event.subaction_path ==
             right_controller_subaction_path_) {
    hand = ControllerHitEvent::Hand::kRight;
  } else {
    // If the subaction path does not match a left or right controller, do
    // not consider this InputActionEvent to belong to a controller.
    // Instead, just send out InputActionUpdateEvents.
    SendInputActionStateChangedEvents(input_action_event);
    return;
  }

  // Identify the raycast action name.
  std::optional<InputActionState<Transform<float>>> raycast_input_action_state =
      std::nullopt;
  for (const auto& [action_name, input_action_state] :
       input_action_event.action_name_to_input_action_state) {
    if (action_name == raycast_action_name_) {
      raycast_input_action_state =
          absl::get<InputActionState<Transform<float>>>(input_action_state);
      break;
    }
  }
  // If this InputActionEvent does not have the Transform action referenced to
  // by raycast_action_name_, do not consider it to belong to a controller.
  // Instead, just send out InputActionUpdateEvents.
  if (!raycast_input_action_state.has_value()) {
    SendInputActionStateChangedEvents(input_action_event);
    return;
  }
  // Perform a ray cast and send the results to the Dispatcher.
  Transform<float> transform = raycast_input_action_state->current_state;
  std::vector<RayHit> ray_hits = view_->GetCollisionManager().IntersectAll(
      Ray{transform.translation, transform.rotation * kForward});
  view_->GetDispatcher().Send(ControllerHitEvent(hand, raycast_action_name_,
                                                 ray_hits, input_action_event));
}

void ControllerInputHandler::SendInputActionStateChangedEvents(
    InputActionEvent input_action_event) {
  for (const auto& [action_name_key, input_action_state] :
       input_action_event.action_name_to_input_action_state) {
    std::visit(
        [this, action_name = std::string(action_name_key),
         input_action_event](const auto& generic_input_action_state) {
          SendInputActionStateChangedEvent(action_name,
                                           input_action_event.subaction_path,
                                           generic_input_action_state);
        },
        input_action_state);
  }
}

}  // namespace imp
