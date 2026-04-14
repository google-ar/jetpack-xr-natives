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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_WHEEL_EVENT_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_WHEEL_EVENT_H_

#include "absl/time/time.h"
#include "core/math/vec.h"

namespace imp {

// Represents a wheel event user interaction.
class WheelEvent {
 public:
  WheelEvent(float2 delta, float2 point, absl::Duration elapsed_time);
  ~WheelEvent() {}

  // Duration of time elapsed between system startup and time of event.
  absl::Duration GetElapsedTime() const { return elapsed_time_; }

  // Returns the delta from the Event.
  float2 GetDelta() const { return delta_; }

  // Returns the point the wheel event occurred.
  float2 GetPoint() const { return point_; }

 private:
  // The amount scrolled horizontally and vertically.
  // Positive delta_.x is to the right, negative to the left.
  // Positive delta_.y is away (up) from the user, negative toward the user
  // (down).
  float2 delta_ = float2(0);
  // The point where the wheel event occurred.
  float2 point_ = float2(0);
  // Duration of time elapsed between system startup and time of event.
  absl::Duration elapsed_time_ = absl::ZeroDuration();
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_WHEEL_EVENT_H_
