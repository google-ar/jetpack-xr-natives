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

#include "core/view/framework/gestures/multi_drag_gesture.h"

#include <math.h>

#include <algorithm>
#include <iterator>
#include <vector>

#include "core/input/pointer_event.h"
#include "core/math/almost_equal.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp {

MultiDragGesture::MultiDragGesture(Dispatcher* dispatcher,
                                   GesturePointerUtils* pointer_utils,
                                   const PointerHitEvent& pointer_hit,
                                   std::vector<Pointer::Id> pointer_ids)
    : Gesture(dispatcher, pointer_utils, pointer_hit),
      pointer_ids_(pointer_ids) {
  float2 total_position = kZero2;
  for (const auto& pointer_id : pointer_ids) {
    int index = pointer_hit.event.GetIndexForPointerId(pointer_id);
    const auto& pointer = pointer_hit.event.GetPointer(index);
    total_position += pointer.point;
    pointer_start_positions_.push_back(pointer.point);
  }
  centroid_start_position_ = centroid_current_position_ =
      total_position / pointer_ids.size();
}

MultiDragGesture::CreateFn MultiDragGesture::GetCreateFunction(
    Dispatcher* dispatcher, GesturePointerUtils* pointer_utils) {
  return
      [dispatcher, pointer_utils](const PointerHitEvent& pointer_hit,
                                  absl::Span<const MultiDragGesture> gestures)
          -> absl::optional<MultiDragGesture> {
        if (pointer_hit.event.Type() != PointerEventType::kDown) {
          return absl::nullopt;
        }

        std::vector<Pointer::Id> pointer_ids;
        for (const auto& pointer : pointer_hit.event.GetPointers()) {
          if (pointer_utils->IsPointerRetained(pointer.id)) {
            return absl::nullopt;
          }
          pointer_ids.push_back(pointer.id);
        }

        return pointer_ids.size() >= kMinPointersPerMultiDrag &&
                       pointer_ids.size() <= kMaxPointersPerMultiDrag
                   ? absl::optional<MultiDragGesture>(MultiDragGesture(
                         dispatcher, pointer_utils, pointer_hit, pointer_ids))
                   : absl::nullopt;
      };
}

bool MultiDragGesture::TryStart(const PointerHitEvent& pointer_hit) {
  // A pointer can only be used in one gesture at a time; cancel if any of their
  // IDS are already retained (i.e., pointer has been started in another
  // gesture).
  for (const auto& pointer_id : pointer_ids_) {
    if (GetPointerUtils()->IsPointerRetained(pointer_id)) {
      Cancel(pointer_hit);
      return false;
    }
  }

  PointerEvent event = pointer_hit.event;
  PointerEventType event_type = event.Type();
  if (!EventHasRelevantChangedPointer(pointer_hit)) {
    // If the event has a pointer down that was not previously part of the
    // gesture we should cancel the gesture. i.e. if we have 4 fingers down we
    // want to start the event tracking the 4 finger drag and not a 3 finger
    // drag.
    if (event_type == PointerEventType::kDown) {
      Cancel(pointer_hit);
      return false;
    }

    // No relevant pointers were updated, do not start.
    return false;
  }

  // The potential multi drag gesture should be cancelled and should not start
  // if any of the tracked pointers went up or were cancelled.
  if (event_type == PointerEventType::kUp ||
      event_type == PointerEventType::kCancel) {
    Cancel(pointer_hit);
    return false;
  }

  // If the pointer has not moved, do not start.
  if (event_type != PointerEventType::kMove) {
    return false;
  }

  // If there are other pointers in this event that are not already part of
  // another gesture, do not start a multi drag gesture.
  int changed_pointer_count = event.ChangedPointerCount();
  if (changed_pointer_count > 1) {
    for (int i = 0; i < changed_pointer_count; ++i) {
      int id = event.GetChangedPointer(i).id;
      if (!std::any_of(
              pointer_ids_.begin(), pointer_ids_.end(),
              [id](Pointer::Id pointer_id) { return id != pointer_id; }) &&
          !GetPointerUtils()->IsPointerRetained(id)) {
        return false;
      }
    }
  }

  // Aggregate the positions and thetas to get the centroid and average theta of
  // the gesture.
  std::vector<int> pointer_indices;
  std::transform(pointer_ids_.begin(), pointer_ids_.end(),
                 std::back_inserter(pointer_indices),
                 [event](Pointer::Id pointer_id) {
                   return event.GetIndexForPointerId(pointer_id);
                 });
  float2 total_pointer_position = kZero2;
  float total_pointer_thetas = 0.0f;
  for (size_t i = 0; i < pointer_indices.size(); ++i) {
    int pointer_idx = pointer_indices[i];
    float2 pointer_new_position = event.GetPointer(pointer_idx).point;
    float2 delta_from_pointer_start =
        pointer_new_position - pointer_start_positions_[i];
    float pointer_distance_travelled = length(delta_from_pointer_start);
    // If any individual pointer movement did not exceed the start threshold, do
    // not start.
    if (pointer_distance_travelled < kDragStartThresholdPixels) {
      return false;
    }

    total_pointer_thetas +=
        atan2(delta_from_pointer_start.y, delta_from_pointer_start.x);
    total_pointer_position += pointer_new_position;
  }

  // If thetas of pointers not aligned with centroid do not start.
  float2 delta_from_centroid_start =
      (total_pointer_position / pointer_ids_.size()) - centroid_start_position_;
  float centroid_theta =
      atan2(delta_from_centroid_start.y, delta_from_centroid_start.x);
  if (abs(((total_pointer_thetas / pointer_ids_.size()) - centroid_theta) /
          centroid_theta) > kErrorTolerance) {
    return false;
  }

  // Multidrag gesture detected
  std::transform(pointer_ids_.begin(), pointer_ids_.end(),
                 std::back_inserter(pointer_retainers_),
                 [this](Pointer::Id pointer_id) {
                   return GetPointerUtils()->RetainPointer(pointer_id);
                 });

  float2 delta = TryUpdateCentroidPosition(pointer_hit).value_or(float2(0, 0));
  GetDispatcher().Send(MultiDragGesture::StartEvent(
      GetId(), MakeCancelFn(this, pointer_hit), pointer_ids_,
      pointer_ids_.size(), centroid_start_position_, delta));

  return true;
}

void MultiDragGesture::OnUpdate(const PointerHitEvent& pointer_hit) {
  const PointerEvent& event = pointer_hit.event;
  const PointerEventType event_type = event.Type();
  // If the event has a pointer down that was not previously part of the gesture
  // we should cancel the gesture.
  if (!EventHasRelevantChangedPointer(pointer_hit)) {
    if (event_type == PointerEventType::kDown) {
      Cancel(pointer_hit);
    }
  } else {
    switch (event.Type()) {
      case PointerEventType::kMove:
        UpdateCentroidPositionAndSendEvent(pointer_hit);
        break;
      case PointerEventType::kUp:
        Finish(pointer_hit);
        break;
      case PointerEventType::kCancel:
        Cancel(pointer_hit);
        break;
      case PointerEventType::kWheel:
      case PointerEventType::kHover:
      case PointerEventType::kEnterView:
      case PointerEventType::kLeaveView:
        break;
      // Handled in previous condition, should not happen
      case PointerEventType::kDown:
        assert(false);
        break;
    }
  }
}

void MultiDragGesture::OnFinish(const PointerHitEvent& pointer_hit) {
  for (auto& pointer_retainer : pointer_retainers_) {
    pointer_retainer.ReleasePointer();
  }

  // Do a final update of the position.
  // This will send a final UpdateEvent if the position has changed since the
  // previous kMove event.
  UpdateCentroidPositionAndSendEvent(pointer_hit);

  GetDispatcher().Send(MultiDragGesture::FinishEvent(
      GetId(), MakeCancelFn(this, pointer_hit), pointer_ids_,
      pointer_ids_.size(), centroid_current_position_, Cancelled()));
}

void MultiDragGesture::OnCancel() {}

absl::optional<float2> MultiDragGesture::TryUpdateCentroidPosition(
    const PointerHitEvent& pointer_hit) {
  PointerEvent event = pointer_hit.event;

  float2 total_position = kZero2;
  for (const auto& pointer_id : pointer_ids_) {
    int move_idx = event.GetIndexForChangedPointerId(pointer_id);
    if (move_idx == -1) {
      return absl::nullopt;
    }

    total_position += event.GetChangedPointer(move_idx).point;
  }

  float2 centroid_new_position = total_position / pointer_ids_.size();
  if (AlmostEqual(centroid_new_position, centroid_current_position_)) {
    return absl::nullopt;
  }

  float2 centroid_delta = centroid_new_position - centroid_current_position_;
  centroid_current_position_ = centroid_new_position;
  return centroid_delta;
}

void MultiDragGesture::UpdateCentroidPositionAndSendEvent(
    const PointerHitEvent& pointer_hit) {
  if (auto delta_or = TryUpdateCentroidPosition(pointer_hit)) {
    GetDispatcher().Send(MultiDragGesture::UpdateEvent(
        GetId(), MakeCancelFn(this, pointer_hit), pointer_ids_,
        pointer_ids_.size(), centroid_current_position_, delta_or.value()));
  }
}

bool MultiDragGesture::EventHasRelevantChangedPointer(
    const PointerHitEvent& pointer_hit) {
  // Check if this event has no relevant pointers.
  const PointerEvent& event = pointer_hit.event;
  std::vector<int> changed_pointer_indices;
  std::transform(pointer_ids_.begin(), pointer_ids_.end(),
                 std::back_inserter(changed_pointer_indices),
                 [event](Pointer::Id pointer_id) {
                   return event.GetIndexForChangedPointerId(pointer_id);
                 });
  return std::any_of(changed_pointer_indices.begin(),
                     changed_pointer_indices.end(),
                     [](int pointer_index) { return pointer_index > -1; });
}

}  // namespace imp
