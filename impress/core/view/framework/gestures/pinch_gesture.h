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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_PINCH_GESTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_PINCH_GESTURE_H_

#include <array>
#include <functional>

#include "core/math/vec.h"
#include "core/view/framework/gestures/gesture.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp {

class View;

/** PinchGesture represents a pinching motion across the device screen. */
class PinchGesture : public Gesture {
 public:
  PinchGesture(Dispatcher* dispatcher, GesturePointerUtils* pointer_utils,
               const PointerHitEvent& pointer_hit,
               const std::array<Pointer::Id, 2>& pointer_id);

  using CreateFn = std::function<absl::optional<PinchGesture>(
      const PointerHitEvent& pointer_hit,
      absl::Span<const PinchGesture> gestures)>;
  static CreateFn GetCreateFunction(Dispatcher* dispatcher,
                                    GesturePointerUtils* pointer_utils);

  struct StartEvent : public Event {
    StartEvent(Id id, CancelFn cancel, std::optional<float> gap = std::nullopt,
               std::optional<float2> centroid = std::nullopt)
        : Event(id, cancel), gap(gap), centroid(centroid) {}
    // Gap in pixels between the two pointers.
    std::optional<float> gap;
    // Centroid of the two pointers.
    std::optional<float2> centroid;
  };

  struct UpdateEvent : public Event {
    UpdateEvent(Id id, CancelFn cancel, float gap, float gap_delta,
                std::optional<float2> centroid = std::nullopt)
        : Event(id, cancel),
          gap(gap),
          gap_delta(gap_delta),
          centroid(centroid) {}
    // Gap in pixels between the two pointers.
    float gap;
    // Delta in pixels for change in pointer positions.
    float gap_delta;
    // Centroid of the two pointers.
    std::optional<float2> centroid = std::nullopt;
  };

  struct FinishEvent : public Event {
    FinishEvent(Id id, bool cancelled) : Event(id), cancelled(cancelled) {}
    bool cancelled;
  };

  float2 GetPosition(size_t index) const {
    return index < 2 ? position_[index] : kZero2;
  }

 protected:
  bool TryStart(const PointerHitEvent& pointer_hit) override;
  void OnUpdate(const PointerHitEvent& pointer_hit) override;
  void OnFinish(const PointerHitEvent& pointer_hit) override;
  void OnCancel() override;

 private:
  static constexpr float kSlopPixels = 8;
  static constexpr float kSlopDegrees = 30.0f;
  Pointer::Id pointer_id_[2];
  GesturePointerUtils::ScopedPointerRetainer pointer_retainer_[2];
  float2 start_position_[2];
  float2 position_[2];
  float start_gap_;
  float gap_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_PINCH_GESTURE_H_
