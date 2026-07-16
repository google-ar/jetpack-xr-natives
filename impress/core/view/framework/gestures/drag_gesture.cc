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

#include "core/view/framework/gestures/drag_gesture.h"

#include "absl/types/variant.h"
#include "core/common/platform_helpers.h"
#include "core/input/input_manager.h"
#include "core/input/pointer_event.h"
#include "core/math/math.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"

namespace imp {

DragGesture::DragGesture(Dispatcher* dispatcher,
                         GesturePointerUtils* pointer_utils,
                         const PointerHitEvent& pointer_hit)
    : Gesture(dispatcher, pointer_utils, pointer_hit),
      pointer_id_(pointer_hit.event.GetChangedPointer().id),
      pointer_retainer_(),
      start_position_(pointer_hit.event.GetChangedPointer().point),
      // TODO Handle RayHit and DoubleRayHit variant
      start_hit_(pointer_hit.GetTruncatedRayHit()),
      position_(start_position_) {}

DragGesture::CreateFn DragGesture::GetCreateFunction(
    Dispatcher* dispatcher, GesturePointerUtils* pointer_utils) {
  return [dispatcher, pointer_utils](const PointerHitEvent& pointer_hit,
                                     absl::Span<const DragGesture> gestures) {
    return pointer_hit.event.Type() == PointerEventType::kDown
               ? absl::optional<DragGesture>(
                     DragGesture(dispatcher, pointer_utils, pointer_hit))
               : absl::nullopt;
  };
}

bool DragGesture::TryStart(const PointerHitEvent& pointer_hit) {
  // A pointer can only be used in one gesture at a time; cancel if its ID is
  // already retained (i.e., pointer has been started in another gesture).
  if (GetPointerUtils()->IsPointerRetained(pointer_id_)) {
    Cancel(pointer_hit);
    return false;
  }

  PointerEvent event = pointer_hit.event;
  int current_pointer_index = event.GetIndexForChangedPointerId(pointer_id_);
  // Event does not contain relevant pointer, do not start.
  if (current_pointer_index == -1) {
    return false;
  }

  PointerEventType event_type = event.Type();
  // The potential drag gesture should be cancelled and should not start if the
  // tracked pointer went up or was cancelled.
  if (event.GetChangedPointer(current_pointer_index).id == pointer_id_ &&
      (event_type == PointerEventType::kUp ||
       event_type == PointerEventType::kCancel)) {
    Cancel(pointer_hit);
    return false;
  }

  // If the pointer has not moved, do not start.
  if (event_type != PointerEventType::kMove) {
    return false;
  }

  // If there are other pointers in this event that are not already part of
  // another gesture, do not start a drag gesture.
  int pointer_count = event.ChangedPointerCount();
  if (pointer_count > 1) {
    for (int i = 0; i < pointer_count; i++) {
      int id = event.GetChangedPointer(i).id;
      if (id != pointer_id_ && !GetPointerUtils()->IsPointerRetained(id)) {
        return false;
      }
    }
  }

  // If pointer movement exceeds our start threshold, then start drag gesture.
  float2 new_position = event.GetChangedPointer(current_pointer_index).point;
  float2 delta_from_start = new_position - start_position_;
  float distance = length(delta_from_start);
  if (distance > kDragStartThresholdPixels) {
    pointer_retainer_ = GetPointerUtils()->RetainPointer(pointer_id_);

    float2 delta = TryUpdatePosition(pointer_hit).value_or(float2(0, 0));
    GetDispatcher().Send(
        GetTargetNode(),
        DragGesture::StartEvent(GetId(), MakeCancelFn(this, pointer_hit),
                                GetTargetNode(), pointer_id_, start_position_,
                                delta, start_hit_));
    return true;
  }

  // Pointer movement did not meet drag start threshold, do not start.
  return false;
}

void DragGesture::OnUpdate(const PointerHitEvent& pointer_hit) {
  // Ignore if the pointer_hit doesn't include the pointer_id.
  PointerEvent event = pointer_hit.event;
  int move_index = event.GetIndexForChangedPointerId(pointer_id_);
  if (move_index > -1) {
    switch (event.Type()) {
      case PointerEventType::kMove: {
        UpdatePositionAndSendEvent(pointer_hit);
        break;
      }
      case PointerEventType::kUp: {
        Finish(pointer_hit);
        break;
      }
      case PointerEventType::kCancel: {
        Cancel(pointer_hit);
        break;
      }
      case PointerEventType::kDown:
      case PointerEventType::kWheel:
      case PointerEventType::kHover:
      case PointerEventType::kEnterView:
      case PointerEventType::kLeaveView: {
        break;
      }
    }
  }
}

void DragGesture::OnFinish(const PointerHitEvent& pointer_hit) {
  pointer_retainer_.ReleasePointer();
  assert(pointer_hit.event.GetChangedPointer().id == pointer_id_);

  // Do a final update of the position.
  // This will send a final UpdateEvent if the position has changed since
  // the previous kMove event.
  UpdatePositionAndSendEvent(pointer_hit);

  // Send the finish event.
  GetDispatcher().Send(
      GetTargetNode(),
      DragGesture::FinishEvent(GetId(), GetTargetNode(), pointer_id_, position_,
                               Cancelled()));
}

void DragGesture::OnCancel() {}

absl::optional<float2> DragGesture::TryUpdatePosition(
    const PointerHitEvent& pointer_hit) {
  PointerEvent event = pointer_hit.event;
  int move_index = event.GetIndexForChangedPointerId(pointer_id_);
  if (move_index <= -1) return {};

  Pointer pointer = event.GetChangedPointer(move_index);
  float2 new_position = pointer.point;
  if (AlmostEqual(new_position, position_)) return {};

  float2 delta = new_position - position_;
  position_ = new_position;
  return delta;
}

void DragGesture::UpdatePositionAndSendEvent(
    const PointerHitEvent& pointer_hit) {
  if (auto delta_or = TryUpdatePosition(pointer_hit)) {
    GetDispatcher().Send(
        GetTargetNode(),
        DragGesture::UpdateEvent(GetId(), MakeCancelFn(this, pointer_hit),
                                 GetTargetNode(), pointer_id_, position_,
                                 *delta_or));
  }
}

}  // namespace imp
