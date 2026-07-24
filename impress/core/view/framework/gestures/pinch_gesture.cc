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

#include <array>
#include <cmath>
#include <optional>

#include "absl/types/span.h"
#include "core/input/pointer_event.h"
#include "core/math/almost_equal.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/framework/gestures/gesture.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp {

namespace {
constexpr float kEpsilon = 0.00001f;
}

PinchGesture::PinchGesture(
    Dispatcher* dispatcher, GesturePointerUtils* pointer_utils,
    const PointerHitEvent& pointer_hit,
    const std::array<Pointer::Id, kPointersPerPinch>& pointer_id)
    : Gesture(dispatcher, pointer_utils, pointer_hit), gap_(-1.f) {
  for (int i = 0; i < kPointersPerPinch; ++i) {
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
             -> std::optional<PinchGesture> {
    if (pointer_hit.event.Type() != PointerEventType::kDown &&
        pointer_hit.event.Type() != PointerEventType::kMove) {
      return std::nullopt;
    }

    // We check kMove here because PinchGesture is registered BEFORE DragGesture
    // in GestureManager. When a second finger goes down (kDown), PinchGesture
    // is processed first. If the first finger is still retained by a
    // DragGesture, PinchGesture will skip it. The DragGesture will then process
    // the kDown, cancel itself, and release the finger. Thus, PinchGesture gets
    // a chance to pick up both fingers on the subsequent kMove event.

    // Only create on kMove if no other pinch is active/unstarted.
    if (pointer_hit.event.Type() == PointerEventType::kMove &&
        !gestures.empty()) {
      return std::nullopt;
    }

    int next = 0;
    std::array<Pointer::Id, kPointersPerPinch> pointer_id;
    for (const auto& pointer : pointer_hit.event.GetPointers()) {
      if (pointer_utils->IsPointerRetained(pointer.id)) continue;
      if (next >= kPointersPerPinch) {
        return std::nullopt;
      }
      pointer_id[next] = pointer.id;
      ++next;
    }
    if (next < kPointersPerPinch) {
      return std::nullopt;
    }
    return PinchGesture(dispatcher, pointer_utils, pointer_hit, pointer_id);
  };
}

bool PinchGesture::TryStart(const PointerHitEvent& pointer_hit) {
  // A pointer can only be used in one gesture at a time; cancel if its ID is
  // already retained (i.e., pointer has been started in another gesture).
  for (int i = 0; i < kPointersPerPinch; ++i) {
    if (GetPointerUtils()->IsPointerRetained(pointer_id_[i])) {
      Cancel(pointer_hit);
      return false;
    }
  }

  const PointerEvent& event = pointer_hit.event;
  auto indices = GetPointerIndices(event);
  if (!indices.has_value()) {
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
      if (!HasPointerId(id) && !GetPointerUtils()->IsPointerRetained(id)) {
        return false;
      }
    }
  }

  float2 new_position[kPointersPerPinch];
  float2 delta[kPointersPerPinch];
  bool moved[kPointersPerPinch];
  bool any_moved = false;
  for (int i = 0; i < kPointersPerPinch; ++i) {
    new_position[i] = event.GetPointer((*indices)[i]).point;
    delta[i] = new_position[i] - position_[i];
    moved[i] = length(delta[i]) > kEpsilon;
    any_moved |= moved[i];
  }

  // We relax the start condition to allow the pinch to start if EITHER pointer
  // moves significantly, not requiring both. This better handles cases where
  // one finger "anchors" while the other moves, or transitions from other
  // gestures. Check which pointers have moved significantly.
  if (!any_moved) {
    return false;
  }

  // Update positions for tracking. We do this before axis calculation to match
  // the original behavior of using the new positions for the axis vector.
  for (int i = 0; i < kPointersPerPinch; ++i) {
    position_[i] = new_position[i];
  }

  float2 axis = position_[1] - position_[0];
  float axis_length = length(axis);
  if (axis_length < kEpsilon) {
    return false;  // Pointers are too close or overlapping.
  }
  float2 axis_norm = normalize(axis);

  // Calculate the projection of the movement vector onto the axis connecting
  // the two pointers. This helps distinguish Pinch/Zoom (movement parallel to
  // the axis) from Rotation/Twist (movement perpendicular to the axis).
  float cos_delta[kPointersPerPinch] = {0.0f, 0.0f};
  if (moved[0]) {
    cos_delta[0] = dot(normalize(delta[0]), axis_norm);
  }
  if (moved[1]) {
    cos_delta[1] = dot(normalize(delta[1]), -axis_norm);
  }

  const float threshold = std::cos(imp::ToRadians(kSlopDegrees));
  if (moved[0] && std::abs(cos_delta[0]) < threshold) {
    return false;
  }
  if (moved[1] && std::abs(cos_delta[1]) < threshold) {
    return false;
  }

  // If both moved, they must be cooperative (moving in opposite directions
  // along the axis).
  if (moved[0] && moved[1] && cos_delta[0] * cos_delta[1] < 0) {
    return false;
  }

  float gap = length(new_position[0] - new_position[1]);
  float abs_delta = std::abs(gap - start_gap_);
  if (abs_delta < kSlopPixels) {
    return false;
  }

  gap_ = gap;
  for (int i = 0; i < kPointersPerPinch; ++i) {
    pointer_retainer_[i] = GetPointerUtils()->RetainPointer(pointer_id_[i]);
  }

  float2 centroid = CalculateCentroid();

  GetDispatcher().Send(PinchGesture::StartEvent(
      GetId(), MakeCancelFn(this, pointer_hit), gap, centroid));
  return true;
}

void PinchGesture::OnUpdate(const PointerHitEvent& pointer_hit) {
  const PointerEvent& event = pointer_hit.event;
  auto indices = GetPointerIndices(event);

  if (indices.has_value()) {
    for (int i = 0; i < kPointersPerPinch; ++i) {
      position_[i] = event.GetPointer((*indices)[i]).point;
    }
    float gap = length(position_[0] - position_[1]);
    switch (event.Type()) {
      case PointerEventType::kMove: {
        if (!AlmostEqual(gap, gap_)) {
          float delta = gap - gap_;
          gap_ = gap;

          float2 centroid = CalculateCentroid();

          GetDispatcher().Send(PinchGesture::UpdateEvent(
              GetId(), MakeCancelFn(this, pointer_hit), gap, delta, centroid));
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
  for (int i = 0; i < kPointersPerPinch; ++i) {
    pointer_retainer_[i].ReleasePointer();
  }
  GetDispatcher().Send(PinchGesture::FinishEvent(GetId(), Cancelled()));
}

void PinchGesture::OnCancel() {}

bool PinchGesture::HasPointerId(Pointer::Id id) const {
  for (int i = 0; i < kPointersPerPinch; ++i) {
    if (pointer_id_[i] == id) {
      return true;
    }
  }
  return false;
}

float2 PinchGesture::CalculateCentroid() const {
  float2 centroid(0.0f, 0.0f);
  for (int i = 0; i < kPointersPerPinch; ++i) {
    centroid += position_[i];
  }
  return centroid / static_cast<float>(kPointersPerPinch);
}

std::optional<std::array<int, PinchGesture::kPointersPerPinch>>
PinchGesture::GetPointerIndices(const PointerEvent& event) const {
  std::array<int, kPointersPerPinch> indices;
  for (int i = 0; i < kPointersPerPinch; ++i) {
    indices[i] = event.GetIndexForPointerId(pointer_id_[i]);
    if (indices[i] == -1) {
      return std::nullopt;
    }
  }
  return indices;
}

}  // namespace imp
