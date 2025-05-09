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

#include "core/view/framework/gestures/tap_gesture.h"

#include "core/input/input_manager.h"
#include "core/input/pointer_event.h"
#include "core/math/quat.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"

namespace imp {

TapGesture::TapGesture(Dispatcher* dispatcher,
                       GesturePointerUtils* pointer_utils,
                       const PointerHitEvent& pointer_hit)
    : Gesture(dispatcher, pointer_utils, pointer_hit),
      start_time_(pointer_hit.event.ElapsedTime()),
      start_position_(pointer_hit.event.GetChangedPointer().point),
      max_down_count_(0),
      down_count_(0) {}

TapGesture::CreateFn TapGesture::GetCreateFunction(
    Dispatcher* dispatcher, GesturePointerUtils* pointer_utils) {
  return [dispatcher, pointer_utils](const PointerHitEvent& pointer_hit,
                                     absl::Span<const TapGesture> gestures) {
    if (pointer_hit.event.Type() != PointerEventType::kDown ||
        !gestures.empty())
      return absl::optional<TapGesture>(absl::nullopt);
    return absl::optional<TapGesture>(
        TapGesture(dispatcher, pointer_utils, pointer_hit));
  };
}

bool TapGesture::TryStart(const PointerHitEvent& pointer_hit) {
  PointerEvent event = pointer_hit.event;
  PointerEventType event_type = event.Type();

  if (event_type == PointerEventType::kDown) {
    if (down_count_ < max_down_count_) {
      // This gesture has already started releasing.
      Cancel(pointer_hit);
      return false;
    }
    if (max_down_count_ >= kMaxPointersPerTap) {
      // Too many pointers to track.
      Cancel(pointer_hit);
      return false;
    }
    pointers_[max_down_count_++] = pointer_hit.event.GetChangedPointer().id;
    down_count_++;
  } else if (event_type == PointerEventType::kUp) {
    for (int i = 0; i < max_down_count_; i++) {
      if (event.GetIndexForPointerId(pointers_[i]) != -1) {
        --down_count_;
        break;
      }
    }
  } else if (event_type == PointerEventType::kCancel) {
    for (int i = 0; i < max_down_count_; i++) {
      if (event.GetIndexForPointerId(pointers_[i]) != -1) {
        Cancel(pointer_hit);
        return false;
      }
    }
  }

  auto elapsed = event.ElapsedTime() - start_time_;
  if (elapsed > kTapThresholdMs) {
    Cancel(pointer_hit);
    return false;
  }
  for (int i = 0; i < max_down_count_; i++) {
    if (GetPointerUtils()->IsPointerRetained(pointers_[i])) {
      // Someone else grabbed these pointers.
      Cancel(pointer_hit);
      return false;
    }
  }
  if (!max_down_count_ || down_count_) {
    return false;
  }
  GetDispatcher().Send(
      GetTargetNode(),
      TapGesture::TapEvent(GetId(), event_type, GetTargetNode(),
                           start_position_, max_down_count_,
                           GetAllIntersectingNodes()));
  Finish(pointer_hit);
  // If we return true here, the state will be reset to Started, but this
  // gesture is already finished.
  return false;
}

void TapGesture::OnUpdate(const PointerHitEvent&) {}
void TapGesture::OnFinish(const PointerHitEvent&) {}
void TapGesture::OnCancel() {}
}  // namespace imp
