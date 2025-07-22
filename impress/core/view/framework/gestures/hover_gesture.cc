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

#include "core/view/framework/gestures/hover_gesture.h"

#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "core/input/pointer_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/gestures/gesture.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp {

HoverGesture::HoverGesture(Dispatcher* dispatcher,
                           GesturePointerUtils* pointer_utils,
                           const PointerHitEvent& pointer_hit)
    : Gesture(dispatcher, pointer_utils, pointer_hit) {}

HoverGesture::CreateFn HoverGesture::GetCreateFunction(
    Dispatcher* dispatcher, GesturePointerUtils* pointer_utils) {
  return [dispatcher, pointer_utils](const PointerHitEvent& pointer_hit,
                                     absl::Span<const HoverGesture> gestures) {
    if (pointer_hit.event.Type() != PointerEventType::kHover ||
        !pointer_hit.GetHitNode().IsValid() || !gestures.empty()) {
      return absl::optional<HoverGesture>(absl::nullopt);
    }

    return absl::optional<HoverGesture>(
        HoverGesture(dispatcher, pointer_utils, pointer_hit));
  };
}

bool HoverGesture::TryStart(const PointerHitEvent& pointer_hit) {
  PointerEventType event_type = pointer_hit.event.Type();
  GetDispatcher().Send(
      GetTargetNode(),
      HoverEvent(GetId(), event_type, GetTargetNode(),
                 GetAllIntersectingNodes(), HoverState::ENTER));
  return true;
}

void HoverGesture::OnUpdate(const PointerHitEvent& pointer_hit) {
  PointerEventType event_type = pointer_hit.event.Type();
  if (event_type != PointerEventType::kHover) {
    return;
  }

  NodeHandle target_node = pointer_hit.GetHitNode();

  // Complete the Gesture if the user is no longer hovering over the node
  if (target_node != GetTargetNode()) {
    Finish(pointer_hit);
    return;
  }

  GetDispatcher().Send(
      GetTargetNode(),
      HoverEvent(GetId(), event_type, GetTargetNode(),
                 pointer_hit.GetAllIntersectingNodes(), HoverState::STAY));
}

void HoverGesture::OnFinish(const PointerHitEvent& pointer_hit) {
  PointerEventType event_type = pointer_hit.event.Type();
  GetDispatcher().Send(
      GetTargetNode(),
      HoverEvent(GetId(), event_type, GetTargetNode(),
                 pointer_hit.GetAllIntersectingNodes(), HoverState::EXIT));
}

void HoverGesture::OnCancel() {}

}  // namespace imp
