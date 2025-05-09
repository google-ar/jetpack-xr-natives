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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_MULTI_DRAG_GESTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_MULTI_DRAG_GESTURE_H_

#include <functional>
#include <optional>
#include <vector>

#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/framework/gestures/gesture.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp {

class View;

// MultiDragGesture represents a dragging motion with multiple fingers across
// the device screen.
class MultiDragGesture : public Gesture {
 public:
  MultiDragGesture(Dispatcher* dispatcher, GesturePointerUtils* pointer_utils,
                   const PointerHitEvent& pointer_hit,
                   std::vector<Pointer::Id> pointer_ids);
  using CreateFn = std::function<absl::optional<MultiDragGesture>(
      const PointerHitEvent& pointer_hit,
      absl::Span<const MultiDragGesture> gestures)>;
  static CreateFn GetCreateFunction(Dispatcher* dispatcher,
                                    GesturePointerUtils* pointer_utils);

  struct MultiDragEvent : public Event {
    MultiDragEvent(Id id, CancelFn cancel, int pointer_count,
                   float2 centroid_position)
        : Event(id, cancel),
          pointer_count(pointer_count),
          centroid_position(centroid_position) {}
    // Number of pointers in this drag gesture.
    int pointer_count;
    // Position in pixels of the initial centroid which initiated the drag
    float2 centroid_position;
  };

  struct StartEvent : public MultiDragEvent {
    StartEvent(Id id, CancelFn cancel, std::vector<Pointer::Id> pointer_ids,
               int pointer_count, float2 centroid_position,
               float2 activation_delta)
        : MultiDragEvent(id, cancel, pointer_count, centroid_position),
          activation_delta(activation_delta) {}
    // Delta in pixels from the initial centroid to the activation point.
    float2 activation_delta;
  };

  struct UpdateEvent : public MultiDragEvent {
    UpdateEvent(Id id, CancelFn cancel, std::vector<Pointer::Id> pointer_ids,
                int pointer_count, float2 centroid_position,
                float2 centroid_delta)
        : MultiDragEvent(id, cancel, pointer_count, centroid_position),
          centroid_delta(centroid_delta) {}
    // Delta in pixels between the new centroid position and the previous one.
    float2 centroid_delta;
  };

  struct FinishEvent : public MultiDragEvent {
    FinishEvent(Id id, CancelFn cancel, std::vector<Pointer::Id> pointer_ids,
                int pointer_count, float2 centroid_position, bool cancelled)
        : MultiDragEvent(id, cancel, pointer_count, centroid_position),
          cancelled(cancelled) {}
    bool cancelled;
  };

  static constexpr size_t kMinPointersPerMultiDrag = 2;
  static constexpr size_t kMaxPointersPerMultiDrag = 4;

 protected:
  bool TryStart(const PointerHitEvent& pointer_hit) override;
  void OnUpdate(const PointerHitEvent& pointer_hit) override;
  void OnFinish(const PointerHitEvent& pointer_hit) override;
  void OnCancel() override;

 private:
  // Update centroid_current_position_ and return (dx, dy) since last update.
  absl::optional<float2> TryUpdateCentroidPosition(
      const PointerHitEvent& pointer_hit);
  void UpdateCentroidPositionAndSendEvent(const PointerHitEvent& pointer_hit);
  bool EventHasRelevantChangedPointer(const PointerHitEvent& pointer_hit);

  static constexpr float kDragStartThresholdPixels = 16.0f;
  static constexpr float kErrorTolerance = 0.05f;

  // The ids of each pointer
  std::vector<Pointer::Id> pointer_ids_;
  // Start position of the pointers used to calculate the slope that they follow
  // to determine if the event should start or not.
  std::vector<float2> pointer_start_positions_;
  std::vector<GesturePointerUtils::ScopedPointerRetainer> pointer_retainers_;
  // The start and current position of the centroid
  float2 centroid_start_position_;
  float2 centroid_current_position_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_MULTI_DRAG_GESTURE_H_
