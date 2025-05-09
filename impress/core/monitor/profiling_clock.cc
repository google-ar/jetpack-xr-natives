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

#include "core/monitor/profiling_clock.h"

#include <time.h>

#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "mediapipe/framework/deps/clock.h"

namespace imp {

absl::Time ProfilingClock::GetMonotonicClockTime() {
  timespec timespec;
  int monotonic_clock_unsupported = 1;
#ifdef CLOCK_MONOTONIC
  monotonic_clock_unsupported = clock_gettime(CLOCK_MONOTONIC, &timespec);
#endif

  if (monotonic_clock_unsupported) {
    return mediapipe::Clock::RealClock()->TimeNow();
  }

  return absl::TimeFromTimespec(timespec);
}

absl::Time ProfilingClock::TimeNow() { return GetMonotonicClockTime(); }

void ProfilingClock::Sleep(absl::Duration d) {
  mediapipe::Clock::RealClock()->Sleep(d);
}

void ProfilingClock::SleepUntil(absl::Time wakeup_time) {
  mediapipe::Clock::RealClock()->SleepUntil(wakeup_time);
}

bool ProfilingClock::AwaitWithDeadline(absl::Mutex* mu,
                                       const absl::Condition& cond,
                                       absl::Time deadline) {
  return mu->AwaitWithDeadline(cond, deadline);
}
}  // namespace imp
