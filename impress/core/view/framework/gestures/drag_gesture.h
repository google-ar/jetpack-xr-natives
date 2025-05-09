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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_DRAG_GESTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_DRAG_GESTURE_H_

#include <functional>
#include <optional>

#include "core/math/vec.h"
#include "core/view/framework/gestures/gesture.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp {

class View;

// DragGesture represents a dragging motion with a single finger across the
// device screen.
class DragGesture : public Gesture {
 public:
  DragGesture(Dispatcher* dispatcher, GesturePointerUtils* pointer_utils,
              const PointerHitEvent& pointer_hit);
  using CreateFn = std::function<absl::optional<DragGesture>(
      const PointerHitEvent& pointer_hit,
      absl::Span<const DragGesture> gestures)>;
  static CreateFn GetCreateFunction(Dispatcher* dispatcher,
                                    GesturePointerUtils* pointer_utils);

  struct DragEvent : public Event {
    DragEvent(Id id, CancelFn cancel, NodeHandle target, Pointer::Id pointer,
              float2 position)
        : Event(id, cancel),
          target(target),
          pointer(pointer),
          position(position) {}

    NodeHandle target;  // Invalid if nothing was targeted.
    // The id of the pointer (i.e. left or right click).
    Pointer::Id pointer;
    // Position in pixels of the initial touch which initiated the drag
    float2 position;
  };

  struct StartEvent : public DragEvent {
    StartEvent(Id id, CancelFn cancel, NodeHandle target_node,
               Pointer::Id pointer_id, float2 pointer_position,
               float2 activation_delta, absl::optional<RayHit> start_hit)
        : DragEvent(id, cancel, target_node, pointer_id, pointer_position),
          activation_delta(activation_delta),
          start_hit(start_hit) {}
    // Delta in pixels from the initial touch to the activation point.
    float2 activation_delta;
    absl::optional<RayHit> start_hit;  // From the initial tap-down.
  };

  struct UpdateEvent : public DragEvent {
    UpdateEvent(Id id, CancelFn cancel, NodeHandle target_node,
                Pointer::Id pointer_id, float2 pointer_position, float2 delta)
        : DragEvent(id, cancel, target_node, pointer_id, pointer_position),
          delta(delta) {}
    // Delta in pixels between the new position and the previous one.
    float2 delta;
  };

  struct FinishEvent : public DragEvent {
    FinishEvent(Id id, NodeHandle target_node, Pointer::Id pointer_id,
                float2 pointer_position, bool cancelled)
        : DragEvent(id, nullptr, target_node, pointer_id, pointer_position),
          cancelled(cancelled) {}
    bool cancelled;
  };

  float2 GetStartPosition() const { return start_position_; }
  float2 GetPosition() const { return position_; }

 protected:
  bool TryStart(const PointerHitEvent& pointer_hit) override;
  void OnUpdate(const PointerHitEvent& pointer_hit) override;
  void OnFinish(const PointerHitEvent& pointer_hit) override;
  void OnCancel() override;

 private:
  absl::optional<float2> TryUpdatePosition(const PointerHitEvent& pointer_hit);
  void UpdatePositionAndSendEvent(const PointerHitEvent& pointer_hit);
  static constexpr float kDragStartThresholdPixels = 16.0f;
  static constexpr absl::Duration kDragStartThresholdDuration =
      absl::Milliseconds(250);
  Pointer::Id pointer_id_;
  GesturePointerUtils::ScopedPointerRetainer pointer_retainer_;
  float2 start_position_;
  absl::Duration start_elapsed_time_;
  absl::optional<RayHit> start_hit_;
  float2 position_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_DRAG_GESTURE_H_
