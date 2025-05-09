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

#ifndef THIRD_PARTY_IMPRESS_CORE_ACTIONS_CONTROLLER_EVENTS_H_
#define THIRD_PARTY_IMPRESS_CORE_ACTIONS_CONTROLLER_EVENTS_H_

#include <optional>
#include <string>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/actions/action_config.h"
#include "core/actions/input_action_event.h"
#include "core/collision/ray.h"
#include "core/math/transform.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

// ControllerHitEvent is a Dispatcher Event that contains:
//   - The results of a ray cast associated with the controller pose.
//   - All InputActionEvents associated with the controller.
//
// Note: All InputActionEvents with the same subaction path *and* action set are
// considered by ControllerInputHandler to be on the same controller.
//
// Example:
//
// GetDispatcher().Connect(
//     [this](const ControllerHitEvent& event) mutable {
//       // Determine Left/Right hand
//       if (event.GetHand() == ControllerHitEvent::Hand::kLeft) {
//         IMP_LOG(imp::ERROR) << "Left hand!";
//       } else {
//         IMP_LOG(imp::ERROR) << "Right hand!";
//       }
//       // Get "select" button state.
//       InputActionState<bool> button_state =
//           event.GetInputActionState<bool>(kDefaultSelectActionName);
//       if (button_state.current_state) {
//         IMP_LOG(imp::ERROR) << "Select button pressed!";
//       }
//       if (button_state.has_changed_since_last_sync) {
//         IMP_LOG(imp::ERROR) << "Select button state has changed since last sync.";
//       }
//       // Get the RayHit.
//       if (event.GetHitNode()) {
//         debug_draw::Global().SphereLines(
//             event.GetHit()->world_point, 1.0f,
//             debug_draw::GetColor(debug_draw::DebugColor::kIndigo));
//       }
//       // Get the controller Ray.
//       Ray ray = event.GetControllerRay();
//       debug_draw::Global().Line(
//           ray.origin, ray.origin + ray.direction * 1.0f,
//           debug_draw::GetColor(debug_draw::DebugColor::kCyan));
//     },
//     this);
//
class ControllerHitEvent : public Event {
 public:
  enum class Hand {
    kUnknown = 0,
    kLeft = 1,
    kRight = 2,
  };

  // Creates a new ControllerHitEvent.
  // raycast_action_name is the name of the Transform<float> input action that
  // was used to create the ray cast.
  ControllerHitEvent(Hand hand, absl::string_view raycast_action_name,
                     std::vector<RayHit> hits,
                     InputActionEvent input_action_event)
      : hits_(hits),
        hand_(hand),
        input_action_event_(input_action_event),
        raycast_action_name_(std::string(raycast_action_name)) {}

  const InputActionStateVariant& GetInputActionStateVariant(
      absl::string_view action_name) const {
    return input_action_event_.GetInputActionStateVariant(
        std::string(action_name));
  }

  // Returns an InputActionState associated with this controller.
  template <typename T>
  const InputActionState<T>& GetInputActionState(
      absl::string_view action_name) const {
    return input_action_event_.GetInputActionState<T>(std::string(action_name));
  }

  // Returns if a action is present for reading on this ControllerHitEvent.
  // For example, some OpenXR controllers may not have "back" buttons.
  bool HasInputAction(absl::string_view action_name) const {
    return input_action_event_.action_name_to_input_action_state.contains(
        action_name);
  }

  template <typename T>
  absl::optional<T> GetInputActionCurrentState(
      absl::string_view action_name) const {
    if (!HasInputAction(action_name)) {
      return std::nullopt;
    }
    return GetInputActionState<T>(action_name).current_state;
  }

  // Returns the closest node that was hit using the controller ray cast.
  std::optional<RayHit> GetHit() const;

  // Returns the NodeHandle associated with the closest RayHit.
  NodeHandle GetHitNode() const;

  // Returns the Transform<float> of the controller.
  const InputActionState<Transform<float>>& GetControllerInputActionState()
      const;

  // Returns the Transform<float> of the controller.
  const Transform<float>& GetControllerTransform() const;

  // Returns the forward Ray of the controller.
  Ray GetControllerRay() const;

  // Returns a vector containing the associated NodeHandle for each RayHit
  // ray hits. Sorted by increasing distance of the original ray hit.
  std::vector<NodeHandle> GetAllIntersectingNodes() const;

  const Hand& GetHand() const;

 private:
  // All hits generated by the ray cast.
  const std::vector<RayHit> hits_;
  const Hand hand_ = Hand::kUnknown;
  // The InputActionEvent from which this controller was generalized.
  const InputActionEvent input_action_event_;
  const std::string raycast_action_name_;
};

// InputActionStateChangedEvent represents the event of a standalone input
// action state changing. If an InputActionState does not belong to the left
// or right controller as identified by ControllerInputHandler, it will be
// wrapped in this event and sent to the Dispatcher.
template <typename T>
struct InputActionStateChangedEvent : public Event {
 public:
  explicit InputActionStateChangedEvent<T>(absl::string_view action_set_name,
                                           absl::string_view subaction_path,
                                           InputActionState<T> state)
      : action_set_name(action_set_name),
        subaction_path(subaction_path),
        input_action_state(state) {}
  std::string action_set_name;
  std::string subaction_path;
  InputActionState<T> input_action_state;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ACTIONS_CONTROLLER_EVENTS_H_
