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

#ifndef JETPACK_XR_NATIVES_OPENXR_OPENXR_MANAGER_CLOCK_H_
#define JETPACK_XR_NATIVES_OPENXR_OPENXR_MANAGER_CLOCK_H_

#include <time.h>

#include "absl/synchronization/mutex.h"
namespace androidx::xr::openxr {

// An interface for the clock used by the OpenXrManager. This is used to allow
// for mocking the clock in tests.
class OpenXrManagerClockInterface {
 public:
  virtual ~OpenXrManagerClockInterface() = default;

  // Returns the current time using the system's monotonic clock.
  virtual timespec TimeNow() = 0;

  // Waits until the condition is true or the deadline has been reached.
  virtual void AwaitWithDeadline(absl::Mutex* mu, const bool* bool_cond,
                                 const timespec& deadline) = 0;
};

// Implementation of the OpenXrManagerClockInterface that uses the system clock.
class OpenXrManagerClock : public OpenXrManagerClockInterface {
 public:
  OpenXrManagerClock() = default;
  timespec TimeNow() override;
  void AwaitWithDeadline(absl::Mutex* mu, const bool* bool_cond,
                         const timespec& deadline) override;
};

}  // namespace androidx::xr::openxr

#endif  // JETPACK_XR_NATIVES_OPENXR_OPENXR_MANAGER_CLOCK_H_
