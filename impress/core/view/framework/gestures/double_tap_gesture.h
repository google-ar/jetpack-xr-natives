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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_DOUBLE_TAP_GESTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_DOUBLE_TAP_GESTURE_H_

#include "core/math/vec.h"
#include "core/view/framework/gestures/gesture.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp {

class View;

/** DoubleTapGesture represents a double tapping action. */
class DoubleTapGesture : public Gesture {
 public:
  DoubleTapGesture(Dispatcher* dispatcher, GesturePointerUtils* pointer_utils,
                   const PointerHitEvent& pointer_hit);
  using CreateFn = std::function<absl::optional<DoubleTapGesture>(
      const PointerHitEvent& pointer_hit,
      absl::Span<const DoubleTapGesture> gestures)>;
  static CreateFn GetCreateFunction(Dispatcher* dispatcher,
                                    GesturePointerUtils* pointer_utils);

  struct TapEvent : public Event {
    TapEvent() {}
    TapEvent(Id id, PointerEventType type, NodeHandle target, float2 position)
        : Event(id), type(type), target(target), position(position) {}
    PointerEventType type;
    NodeHandle target;  // Invalid if nothing was targeted.
    float2 position;
  };

 protected:
  bool TryStart(const PointerHitEvent& pointer_hit) override;
  void OnUpdate(const PointerHitEvent& pointer_hit) override;
  void OnFinish(const PointerHitEvent& pointer_hit) override;
  void OnCancel() override;

 private:
  static constexpr float kDoubleTapThresholdPixels = 16.0f;
  static constexpr absl::Duration kDoubleTapThresholdMs =
      absl::Milliseconds(400);

  Pointer::Id pointer_id_;
  absl::Duration start_time_;
  float2 start_position_;
  bool saw_up_ = false;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_DOUBLE_TAP_GESTURE_H_
