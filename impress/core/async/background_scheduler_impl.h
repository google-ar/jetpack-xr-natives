// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_BACKGROUND_SCHEDULER_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_BACKGROUND_SCHEDULER_IMPL_H_

#include <cstdint>

#include "absl/status/status.h"
#include "core/async/background_scheduler.h"
#include "core/async/future.h"
#include "core/common/invocable.h"

namespace imp {

// Schedules work on background executor.
class BackgroundSchedulerImpl : public BackgroundScheduler {
 public:
  BackgroundSchedulerImpl() = default;
  ~BackgroundSchedulerImpl() override;

  // Schedules a function to be executed on the background thread.
  // Shall be accessed only on the foreground thread only.
  void Schedule(imp::Invocable<absl::Status()> fn) override;

 private:
  // Represents a chain of operations that are scheduled to be executed on
  // the background thread.
  //
  // Accessed via `Schedule` method only and only on the foreground thread.
  Future<absl::Status> pending_ops_ = Future<absl::Status>(absl::OkStatus());

  // This is used to determine when it's time to use kScheduleAlways mode to
  // prevent infinite callstacks. See `Schedule` method for more details.
  //
  // Using signed int as per (broken link).
  int64_t total_ops_count_ = 0;

  // Obtained results from `future_benchmark_test.cc` shows that 64 is sweet
  // spot both for XR device and Desktop.
  static constexpr int64_t kScheduleAlwaysEveryN = 64;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_BACKGROUND_SCHEDULER_IMPL_H_
