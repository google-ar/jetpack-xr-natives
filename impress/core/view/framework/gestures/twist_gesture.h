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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_TWIST_GESTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_TWIST_GESTURE_H_

#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/view/framework/gestures/gesture.h"
#include "core/view/framework/gestures/gesture_pointer_utils.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp {

class View;

/** TwistGesture represents a pinching motion across the device screen. */
class TwistGesture : public Gesture {
 public:
  TwistGesture(Dispatcher* dispatcher, GesturePointerUtils* pointer_utils,
               const PointerHitEvent& pointer_hit,
               const std::array<Pointer::Id, 2>& pointer_id);

  using CreateFn = std::function<absl::optional<TwistGesture>(
      const PointerHitEvent& pointer_hit,
      absl::Span<const TwistGesture> gestures)>;
  static CreateFn GetCreateFunction(Dispatcher* dispatcher,
                                    GesturePointerUtils* pointer_utils);

  struct StartEvent : public Event {
    StartEvent(Id id, CancelFn cancel, std::optional<float> gap = std::nullopt,
               std::optional<float2> centroid = std::nullopt)
        : Event(id, cancel), gap(gap), centroid(centroid) {}
    std::optional<float> gap;
    std::optional<float2> centroid;
  };

  struct UpdateEvent : public Event {
    UpdateEvent(Id id, CancelFn cancel, float delta_radians,
                std::optional<float> gap = std::nullopt,
                std::optional<float2> centroid = std::nullopt)
        : Event(id, cancel),
          delta_radians(delta_radians),
          gap(gap),
          centroid(centroid) {}
    float delta_radians;
    std::optional<float> gap;
    std::optional<float2> centroid;
  };

  struct FinishEvent : public Event {
    FinishEvent(Id id, bool cancelled) : Event(id), cancelled(cancelled) {}
    bool cancelled;
  };

 protected:
  bool TryStart(const PointerHitEvent& pointer_hit) override;
  void OnUpdate(const PointerHitEvent& pointer_hit) override;
  void OnFinish(const PointerHitEvent& pointer_hit) override;
  void OnCancel() override;

 private:
  static constexpr float kSlopRotationRadians = ToRadians(15.0f);
  Pointer::Id pointer_id_[2];
  GesturePointerUtils::ScopedPointerRetainer pointer_retainer_[2];
  float2 start_positions_[2];
  float2 previous_positions_[2];
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_GESTURES_TWIST_GESTURE_H_
