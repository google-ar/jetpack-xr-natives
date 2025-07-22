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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_HOVER_GESTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_HOVER_GESTURE_H_

#include <cstdint>
#include <functional>
#include <vector>

#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "core/input/pointer_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/framework/gestures/gesture.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp {

class View;

/**
 * HoverGesture represents a mouse hovering action on an object in the scene.
 *
 * Uses the raycast from PointerHitEvents as a "hover ray" to determine what
 * node the user is hovering over. Will fire a HoverEvent whenever the user
 * begins/continues/stops hovering over an object in the scene.
 *
 * Variables:
 *  - `target`
 *    - the node being hovered by the user
 *    - will be the closest node that the hover ray initially hits
 *  - `all_intersecting_nodes`
 *    - a list of all nodes in the scene that the hover ray is touching
 *    - the node stored in `target` will also be in this list
 *  - `state`
 *    - the current HoverState event of the HoverEvent
 *
 * Order of HoverState events:
 *  - 1. HoverState::ENTER
 *  - 2. HoverState::STAY
 *  - 3. HoverState::EXIT
 *
 * State Events:
 *  - HoverState::ENTER
 *    - fired on the frame the user begins hovering over a node in the scene
 *    - will pass along the front most node and a list of all nodes that the
 *    hover ray hit to `target` and `all_intersecting_nodes` respectively
 *  - HoverState::STAY
 *    - fired every frame after the user begins hovering over a node in the
 *    scene
 *    - the node stored in `target` should be the same as it was in the most
 *    recent HoverState::ENTER event
 *    - `all_intersecting_nodes` will be updated to match the current list of
 *    nodes the hover ray is hitting when this event is fired
 *  - HoverState::EXIT
 *    - fired on the frame the user stops hovering over the node that trigged
 *    the inital HoverState::ENTER event
 *    - the node in `target` is still be set to the same node that was in the
 *    most recent HoverState::ENTER event
 *    - `all_intersecting_nodes` will be set to the new nodes the hover ray is
 *    now hitting
 *    - if the user is not hovering over anything when the event is triggered,
 *    then `all_intersecting_nodes` will be empty
 */
class HoverGesture : public Gesture {
 public:
  // A State enum used to express the user's current action during a HoverEvent
  enum HoverState : uint32_t {
    // Fired when the user begins hovering over an object in the scene.
    ENTER,
    // Fired every frame the user hovers over an object in the scene.
    STAY,
    // Fired whenever the user stops hovering or switches hover targets.
    EXIT,
  };

  HoverGesture(Dispatcher* dispatcher, GesturePointerUtils* pointer_utils,
               const PointerHitEvent& pointer_hit);
  using CreateFn = std::function<absl::optional<HoverGesture>(
      const PointerHitEvent& pointer_hit,
      absl::Span<const HoverGesture> gestures)>;
  static CreateFn GetCreateFunction(Dispatcher* dispatcher,
                                    GesturePointerUtils* pointer_utils);

  // An Event that is fired whenever the user has started, continued to, or
  // stopped hovering over a node in the scene
  struct HoverEvent : public Event {
    HoverEvent(Id id, PointerEventType type, NodeHandle target,
               std::vector<NodeHandle> all_intersecting_nodes, HoverState state)
        : Event(id),
          target(target),
          all_intersecting_nodes(all_intersecting_nodes),
          state(state) {}
    // the handle of the node that is/was being hovered over
    NodeHandle target;
    // all of the nodes on that were hit by the hover ray in this event
    std::vector<NodeHandle> all_intersecting_nodes;
    // the current hover state
    HoverState state;
  };

 protected:
  bool TryStart(const PointerHitEvent& pointer_hit) override;
  void OnUpdate(const PointerHitEvent& pointer_hit) override;
  void OnFinish(const PointerHitEvent& pointer_hit) override;
  void OnCancel() override;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_HOVER_GESTURE_H_
