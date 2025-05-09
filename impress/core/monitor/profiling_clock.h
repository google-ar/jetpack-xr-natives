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

#ifndef THIRD_PARTY_IMPRESS_CORE_MONITOR_PROFILING_CLOCK_H_
#define THIRD_PARTY_IMPRESS_CORE_MONITOR_PROFILING_CLOCK_H_
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "mediapipe/framework/deps/clock.h"

namespace imp {

// An implementation of the mediapipe::Clock API for use in the monitor library.
class ProfilingClock : public mediapipe::Clock {
 public:
  ProfilingClock() = default;

  ~ProfilingClock() override = default;

  // Provides a monotonic timestamp as a absl::Time.
  // If the monotonic clock is unsupported the fallback is to use absl's
  // real clock.
  static absl::Time GetMonotonicClockTime();

  // The Clock interface (see util/time/clock.h).
  //

  // Returns a time based on clock_gettime(CLOCK_MONOTONIC) if it is available.
  // If it is not available, the fallback is to use Clock::RealClock().
  absl::Time TimeNow() override;

  // Uses the mediapipe::Clock::RealClock() implementation.
  void Sleep(absl::Duration d) override;

  // Uses the mediapipe::Clock::RealClock() implementation.
  void SleepUntil(absl::Time wakeup_time) override;

  // Uses the mediapipe::Clock::RealClock() implementation.
  bool AwaitWithDeadline(absl::Mutex* mu, const absl::Condition& cond,
                         absl::Time deadline);

 private:
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MONITOR_PROFILING_CLOCK_H_
