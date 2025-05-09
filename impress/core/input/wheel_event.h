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

#include <vector>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "filament/libs/math/include/math/vec2.h"
#include "core/math/vec.h"

namespace imp {

// Represents a wheel event user interaction.
class WheelEvent {
 public:
  WheelEvent(const float delta, absl::Duration elapsed_time);
  ~WheelEvent() {}

  // Duration of time elapsed between system startup and time of event.
  absl::Duration GetElapsedTime() const { return elapsed_time_; }

  // Returns the delta from the Event.
  float GetDelta() const { return delta_; }

 private:
  // The scroll wheel vertical delta value, or how much the wheel.
  float delta_ = 0;
  // Duration of time elapsed between system startup and time of event.
  absl::Duration elapsed_time_ = absl::ZeroDuration();
  // Allow only the InputManager to set the delta and elapsed time.
  friend class InputManager;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_WHEEL_EVENT_H_
