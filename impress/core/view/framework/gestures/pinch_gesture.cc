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

#include "core/view/framework/gestures/pinch_gesture.h"

#include <cmath>
#include <vector>

#include "core/common/platform_helpers.h"
#include "core/input/pointer_event.h"
#include "core/math/almost_equal.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"

namespace imp {

namespace {
constexpr float kEpsilon = 0.00001f;
}

PinchGesture::PinchGesture(Dispatcher* dispatcher,
                           GesturePointerUtils* pointer_utils,
                           const PointerHitEvent& pointer_hit,
                           const std::array<Pointer::Id, 2>& pointer_id)
    : Gesture(dispatcher, pointer_utils, pointer_hit), gap_(-1.f) {
  for (int i = 0; i < 2; ++i) {
    pointer_id_[i] = pointer_id[i];
    int index = pointer_hit.event.GetIndexForPointerId(pointer_id[i]);
    const auto& pointer = pointer_hit.event.GetPointer(index);
    start_position_[i] = pointer.point;
    position_[i] = pointer.point;
  }
  start_gap_ = length(start_position_[0] - start_position_[1]);
}

PinchGesture::CreateFn PinchGesture::GetCreateFunction(
    Dispatcher* dispatcher, GesturePointerUtils* pointer_utils) {
  return [dispatcher, pointer_utils](const PointerHitEvent& pointer_hit,
                                     absl::Span<const PinchGesture> gestures)
             -> absl::optional<PinchGesture> {
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
    return PinchGesture(dispatcher, pointer_utils, pointer_hit, pointer_id);
  };
}

bool PinchGesture::TryStart(const PointerHitEvent& pointer_hit) {
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
  // The potential pinch gesture should be cancelled and should not start if
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
  // another gesture, do not start a pinch gesture.
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

  float2 new_position[2] = {event.GetPointer(current_pointer_index[0]).point,
                            event.GetPointer(current_pointer_index[1]).point};
  float2 delta[2] = {new_position[0] - position_[0],
                     new_position[1] - position_[1]};
  position_[0] = new_position[0];
  position_[1] = new_position[1];

  if (length(delta[0]) < kEpsilon || length(delta[1]) < kEpsilon) {
    return false;
  }

  float cos_delta[2] = {
      dot(normalize(delta[0]), normalize(position_[1] - position_[0])),
      dot(normalize(delta[1]), normalize(position_[0] - position_[1]))};
  const float threshold = std::cos(imp::ToRadians(kSlopDegrees));
  if (std::abs(cos_delta[0]) < threshold ||
      std::abs(cos_delta[1]) < threshold) {
    return false;
  }

  // If the sign of the cos_delta is different then it is not a pinch gesture
  if (cos_delta[0] * cos_delta[1] < 0) {
    return false;
  }

  float gap = length(new_position[0] - new_position[1]);
  float abs_delta = std::abs(gap - start_gap_);
  if (abs_delta < kSlopPixels) {
    return false;
  }

  gap_ = gap;
  pointer_retainer_[0] = GetPointerUtils()->RetainPointer(pointer_id_[0]);
  pointer_retainer_[1] = GetPointerUtils()->RetainPointer(pointer_id_[1]);
  GetDispatcher().Send(
      PinchGesture::StartEvent(GetId(), MakeCancelFn(this, pointer_hit), gap,
                               (position_[0] + position_[1]) * 0.5f));
  return true;
}

void PinchGesture::OnUpdate(const PointerHitEvent& pointer_hit) {
  const PointerEvent& event = pointer_hit.event;
  int indices[2] = {event.GetIndexForPointerId(pointer_id_[0]),
                    event.GetIndexForPointerId(pointer_id_[1])};
  if (indices[0] > -1 && indices[1] > -1) {
    const Pointer* pointers[2] = {&event.GetPointer(indices[0]),
                                  &event.GetPointer(indices[1])};
    position_[0] = pointers[0]->point;
    position_[1] = pointers[1]->point;
    float gap = length(position_[0] - position_[1]);
    switch (event.Type()) {
      case PointerEventType::kMove: {
        if (!AlmostEqual(gap, gap_)) {
          float delta = gap - gap_;
          gap_ = gap;
          GetDispatcher().Send(PinchGesture::UpdateEvent(
              GetId(), MakeCancelFn(this, pointer_hit), gap, delta,
              (position_[0] + position_[1]) * 0.5f));
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

void PinchGesture::OnFinish(const PointerHitEvent& pointer_hit) {
  pointer_retainer_[0].ReleasePointer();
  pointer_retainer_[1].ReleasePointer();
  GetDispatcher().Send(PinchGesture::FinishEvent(GetId(), Cancelled()));
}

void PinchGesture::OnCancel() {}
}  // namespace imp
