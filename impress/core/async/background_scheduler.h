// Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_BACKGROUND_SCHEDULER_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_BACKGROUND_SCHEDULER_H_

#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/synchronization/notification.h"
#include "core/common/invocable.h"

namespace imp {

// Schedules work to be run sequentially.
class BackgroundScheduler {
 public:
  virtual ~BackgroundScheduler() = default;

  virtual void Schedule(imp::Invocable<absl::Status()> fn) = 0;

  // This is blocking call: it waits for all pending work to complete.
  void Drain(absl::Status status = absl::OkStatus()) {
    absl::Notification notification;
    Schedule([&notification, status = std::move(status),
              cleanup = absl::Cleanup(
                  [&notification] { notification.Notify(); })]() mutable {
      notification.Notify();
      std::move(cleanup).Cancel();
      return status;
    });
    notification.WaitForNotification();
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_BACKGROUND_SCHEDULER_H_
