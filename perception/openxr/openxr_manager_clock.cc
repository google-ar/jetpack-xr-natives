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

#include "openxr/openxr_manager_clock.h"

#include <ctime>

#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"

namespace androidx::xr::openxr {
timespec OpenXrManagerClock::TimeNow() {
  timespec system_time;
  // Use the monotonic time to when getting the system time. It is the clock
  // required by the OpenXR spec.
  clock_gettime(CLOCK_MONOTONIC, &system_time);
  return system_time;
}

void OpenXrManagerClock::AwaitWithDeadline(absl::Mutex* mu,
                                           const bool* bool_cond,
                                           const timespec& deadline) {
  absl::Duration timeout =
      absl::TimeFromTimespec(deadline) - absl::TimeFromTimespec(TimeNow());

  mu->AwaitWithTimeout(absl::Condition(bool_cond), timeout);
}

}  // namespace androidx::xr::openxr
