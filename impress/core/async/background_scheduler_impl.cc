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

#include "core/async/background_scheduler_impl.h"

#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/invocable.h"

namespace imp {

BackgroundSchedulerImpl::~BackgroundSchedulerImpl() = default;

void BackgroundSchedulerImpl::Schedule(imp::Invocable<absl::Status()> fn) {
  

  // Once future becomes ready, it stops referencing its parents, so there's no
  // need to manage the chain manually. I.e., it's not possible to have a chain
  // that have 100 completed futures and keep on growing.
  //
  // What actually can happen is that we may have a very long chain of pending
  // futures (think creating and destroying a hundred of textures per frame).
  // Since child futures are called recursively by default, that increases the
  // risk of stack overflow (call stacks with 1000+ frames were observed in
  // stress tests).
  //
  // `future_benchmark_test.cc` shows that using kScheduleAlways slow things
  // down noticeably. Based on obtained results, it's better to inject
  // kScheduleAlways every 64 futures to maintain performance that is similar to
  // default behavior, and keep the size of the call stack reasonable.
  //
  // TODO: (broken link) - re-visit this logic later to make sure it still shows
  // good performance.
  total_ops_count_++;
  const FutureExecutorMode executor_mode =
      (total_ops_count_ % kScheduleAlwaysEveryN == 0)
          ? FutureExecutorMode::kScheduleAlways
          : FutureExecutorMode::kScheduleIfNotOnExecutorThread;

  pending_ops_ = pending_ops_.Then(
      std::move(fn), {
                         .executor = Executor::BackgroundExecutor(),
                         .executor_mode = executor_mode,
                     });
}

}  // namespace imp
