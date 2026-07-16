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

#include "core/view/framework/gestures/double_tap_gesture.h"

#include "core/common/platform_helpers.h"
#include "core/input/input_manager.h"
#include "core/input/pointer_event.h"
#include "core/math/quat.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"

namespace imp {

DoubleTapGesture::DoubleTapGesture(Dispatcher* dispatcher,
                                   GesturePointerUtils* pointer_utils,
                                   const PointerHitEvent& pointer_hit)
    : Gesture(dispatcher, pointer_utils, pointer_hit),
      pointer_id_(pointer_hit.event.GetChangedPointer().id),
      start_time_(pointer_hit.event.ElapsedTime()),
      start_position_(pointer_hit.event.GetChangedPointer().point) {}

DoubleTapGesture::CreateFn DoubleTapGesture::GetCreateFunction(
    Dispatcher* dispatcher, GesturePointerUtils* pointer_utils) {
  return
      [dispatcher, pointer_utils](const PointerHitEvent& pointer_hit,
                                  absl::Span<const DoubleTapGesture> gestures) {
        return pointer_hit.event.Type() == PointerEventType::kDown
                   ? absl::optional<DoubleTapGesture>(DoubleTapGesture(
                         dispatcher, pointer_utils, pointer_hit))
                   : absl::nullopt;
      };
}

bool DoubleTapGesture::TryStart(const PointerHitEvent& pointer_hit) {
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
  // The potential double tap gesture should be cancelled and should not start
  // if the tracked pointer was cancelled.
  if (event.GetChangedPointer(current_pointer_index).id == pointer_id_ &&
      event_type == PointerEventType::kCancel) {
    Cancel(pointer_hit);
    return false;
  }

  if ((event.ElapsedTime() - start_time_) >= kDoubleTapThresholdMs) {
    Cancel(pointer_hit);
    return false;
  }

  if (event_type == PointerEventType::kUp && !saw_up_) {
    saw_up_ = true;
    return false;
  }

  // If there are other pointers in this event that are not already part of
  // another gesture, do not start a double tap gesture.
  int pointer_count = event.ChangedPointerCount();
  if (pointer_count > 1) {
    for (int i = 0; i < pointer_count; i++) {
      int id = event.GetChangedPointer(i).id;
      if (id != pointer_id_ && !GetPointerUtils()->IsPointerRetained(id)) {
        return false;
      }
    }
  }

  // If pointer movement and time are within thresholds, then send double tap.
  float2 new_position = event.GetChangedPointer().point;
  float distance = length(new_position - start_position_);
  auto elapsed = event.ElapsedTime() - start_time_;

  // Dispatches double tap events that end on up and on down.
  if ((event_type == PointerEventType::kUp ||
       event_type == PointerEventType::kDown) &&
      elapsed < kDoubleTapThresholdMs && distance < kDoubleTapThresholdPixels) {
    GetDispatcher().Send(
        GetTargetNode(),
        DoubleTapGesture::TapEvent(GetId(), event_type, GetTargetNode(),
                                   {start_position_, new_position}));
    if (event_type == PointerEventType::kUp) {
      Finish(pointer_hit);
    }
    // If we return true here, the state will be reset to Started, but this
    // gesture is already finished.
    return false;
  }

  // Pointer movement did not meet double tap start threshold, do not start.
  return false;
}

void DoubleTapGesture::OnUpdate(const PointerHitEvent&) {}
void DoubleTapGesture::OnFinish(const PointerHitEvent&) {}
void DoubleTapGesture::OnCancel() {}
}  // namespace imp
