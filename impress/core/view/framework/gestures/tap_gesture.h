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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_TAP_GESTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_TAP_GESTURE_H_

#include <vector>

#include "core/math/vec.h"
#include "core/view/framework/gestures/gesture.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp {

class View;

/** TapGesture represents a tapping action. */
class TapGesture : public Gesture {
 public:
  TapGesture(Dispatcher* dispatcher, GesturePointerUtils* pointer_utils,
             const PointerHitEvent& pointer_hit);
  using CreateFn = std::function<absl::optional<TapGesture>(
      const PointerHitEvent& pointer_hit,
      absl::Span<const TapGesture> gestures)>;
  static CreateFn GetCreateFunction(Dispatcher* dispatcher,
                                    GesturePointerUtils* pointer_utils);

  struct TapEvent : public Event {
    TapEvent(Id id, PointerEventType type, NodeHandle target, float2 position,
             int pointer_count, std::vector<NodeHandle> all_intersecting_nodes)
        : Event(id),
          type(type),
          target(target),
          position(position),
          pointer_count(pointer_count),
          all_intersecting_nodes(all_intersecting_nodes) {}
    PointerEventType type;
    NodeHandle target;  // Invalid if nothing was targeted.
    float2 position;
    int pointer_count;
    std::vector<NodeHandle> all_intersecting_nodes;
  };

  float2 GetStartPosition() const { return start_position_; }

 protected:
  bool TryStart(const PointerHitEvent& pointer_hit) override;
  void OnUpdate(const PointerHitEvent& pointer_hit) override;
  void OnFinish(const PointerHitEvent& pointer_hit) override;
  void OnCancel() override;

 private:
  static constexpr absl::Duration kTapThresholdMs = absl::Milliseconds(250);

  static constexpr size_t kMaxPointersPerTap = 4;
  std::array<Pointer::Id, kMaxPointersPerTap> pointers_;
  absl::Duration start_time_;
  float2 start_position_;
  int32_t max_down_count_;
  int32_t down_count_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_TAP_GESTURE_H_
