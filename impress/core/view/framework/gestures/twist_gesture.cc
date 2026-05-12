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

#include "core/view/framework/gestures/twist_gesture.h"

#include <array>

#include "core/input/pointer_event.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"

namespace imp {

namespace {
constexpr float kEpsilon = 0.00001f;
}

TwistGesture::TwistGesture(Dispatcher* dispatcher,
                           GesturePointerUtils* pointer_utils,
                           const PointerHitEvent& pointer_hit,
                           const std::array<Pointer::Id, 2>& pointer_id)
    : Gesture(dispatcher, pointer_utils, pointer_hit) {
  for (int i = 0; i < 2; ++i) {
    pointer_id_[i] = pointer_id[i];
    int index = pointer_hit.event.GetIndexForPointerId(pointer_id[i]);
    const auto& pointer = pointer_hit.event.GetPointer(index);
    start_positions_[i] = pointer.point;
    previous_positions_[i] = pointer.point;
  }
}

TwistGesture::CreateFn TwistGesture::GetCreateFunction(
    Dispatcher* dispatcher, GesturePointerUtils* pointer_utils) {
  return [dispatcher, pointer_utils](const PointerHitEvent& pointer_hit,
                                     absl::Span<const TwistGesture> gestures)
             -> absl::optional<TwistGesture> {
    if (pointer_hit.event.Type() != PointerEventType::kDown) {
      return absl::nullopt;
    }

    int next = 0;
    std::array<Pointer::Id, 2> pointer_id;
    for (const auto& pointer : pointer_hit.event.GetPointers()) {
      if (pointer_utils->IsPointerRetained(pointer.id)) continue;
      if (next >= 2) {
        return absl::nullopt;
      }
      pointer_id[next] = pointer.id;
      ++next;
    }
    if (next < 2) {
      return absl::nullopt;
    }
    return TwistGesture(dispatcher, pointer_utils, pointer_hit, pointer_id);
  };
}

float CalculateDeltaRadians(float2* current_positions,
                            float2* previous_positions) {
  float2 current_direction =
      normalize(current_positions[0] - current_positions[1]);
  float2 previous_direction =
      normalize(previous_positions[0] - previous_positions[1]);
  float sign = copysignf(1.0f, previous_direction.x * current_direction.y -
                                   previous_direction.y * current_direction.x);
  return RadiansBetween(current_direction, previous_direction) * sign;
}

bool TwistGesture::TryStart(const PointerHitEvent& pointer_hit) {
  // A pointer can only be used in one gesture at a time; cancel if its ID is
  // already retained (i.e., pointer has been started in another gesture).
  if (GetPointerUtils()->IsPointerRetained(pointer_id_[0]) ||
      GetPointerUtils()->IsPointerRetained(pointer_id_[1])) {
    Cancel(pointer_hit);
    return false;
  }

  const PointerEvent& event = pointer_hit.event;
  int current_pointer_index[2] = {event.GetIndexForPointerId(pointer_id_[0]),
                                  event.GetIndexForPointerId(pointer_id_[1])};
  // Event does not contain relevant pointers, do not start.
  if (current_pointer_index[0] == -1 || current_pointer_index[1] == -1) {
    return false;
  }

  PointerEventType event_type = event.Type();
  // The potential twist gesture should be cancelled and should not start if
  // either of the tracked pointers went up or was cancelled.
  if (event_type == PointerEventType::kUp ||
      event_type == PointerEventType::kCancel) {
    Cancel(pointer_hit);
    return false;
  }

  // If the neither pointer has moved, do not start.
  if (event_type != PointerEventType::kMove) {
    return false;
  }

  // If there are other pointers in this event that are not already part of
  // another gesture, do not start a twist gesture.
  int pointer_count = event.ChangedPointerCount();
  if (pointer_count > 1) {
    for (int i = 0; i < pointer_count; i++) {
      int id = event.GetPointer(i).id;
      if (id != pointer_id_[0] && id != pointer_id_[1] &&
          !GetPointerUtils()->IsPointerRetained(id)) {
        return false;
      }
    }
  }

  // Calculate the difference in angle between the current pointer positions
  // and the starting pointer positions.
  float2 positions[2] = {event.GetPointer(current_pointer_index[0]).point,
                         event.GetPointer(current_pointer_index[1]).point};
  float2 delta_positions[2] = {positions[0] - previous_positions_[0],
                               positions[1] - previous_positions_[1]};
  previous_positions_[0] = positions[0];
  previous_positions_[1] = positions[1];

  if (length(delta_positions[0]) < kEpsilon ||
      length(delta_positions[1]) < kEpsilon) {
    return false;
  }

  // Do not start, the difference in angle is too small.
  float delta_radians = CalculateDeltaRadians(positions, start_positions_);
  if (abs(delta_radians) < kSlopRotationRadians) {
    return false;
  }

  // Start the gesture, retain the pointers and send the event.
  pointer_retainer_[0] = GetPointerUtils()->RetainPointer(pointer_id_[0]);
  pointer_retainer_[1] = GetPointerUtils()->RetainPointer(pointer_id_[1]);
  GetDispatcher().Send(
      TwistGesture::StartEvent(GetId(), MakeCancelFn(this, pointer_hit),
                               distance(positions[0], positions[1]),
                               (positions[0] + positions[1]) * 0.5f));

  return true;
}

void TwistGesture::OnUpdate(const PointerHitEvent& pointer_hit) {
  const PointerEvent& event = pointer_hit.event;
  int indices[2] = {event.GetIndexForPointerId(pointer_id_[0]),
                    event.GetIndexForPointerId(pointer_id_[1])};
  if (indices[0] > -1 && indices[1] > -1) {
    switch (event.Type()) {
      case PointerEventType::kMove: {
        // Calculating the difference in angle between the previous move event
        // and send an update event.
        float2 positions[2] = {event.GetPointer(indices[0]).point,
                               event.GetPointer(indices[1]).point};

        float delta_radians =
            CalculateDeltaRadians(positions, previous_positions_);

        previous_positions_[0] = positions[0];
        previous_positions_[1] = positions[1];

        if (!AlmostEqual(delta_radians, 0.0f)) {
          GetDispatcher().Send(TwistGesture::UpdateEvent(
              GetId(), MakeCancelFn(this, pointer_hit), delta_radians,
              distance(positions[0], positions[1]),
              (positions[0] + positions[1]) * 0.5f));
        }
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
      default: {
        Cancel(pointer_hit);
        break;
      }
    }
  }
}

void TwistGesture::OnFinish(const PointerHitEvent& pointer_hit) {
  GetDispatcher().Send(TwistGesture::FinishEvent(GetId(), Cancelled()));
}

void TwistGesture::OnCancel() {}

}  // namespace imp
