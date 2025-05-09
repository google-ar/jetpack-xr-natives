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

#include "core/async/executor_helpers.h"

#include <cassert>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/config.h"

#if IMP_THREADS(GOOGLE3)
#include "thread/thread.h"
#elif IMP_THREADS(STDLIB)
#include <thread>
#endif

namespace imp {

absl::StatusOr<std::unique_ptr<ExecutorsHolder>> TryCreateAndSetExecutors(
    CreateExecutorsFn create_executors_fn) {
  if (Executor::ForegroundExecutor() != nullptr) {
    return absl::AlreadyExistsError(
        "Cannot create foreground executor, one already exists.");
  }

  if (Executor::BackgroundExecutor() != nullptr) {
    return absl::AlreadyExistsError(
        "Cannot create background executor, one already exists.");
  }

  ExecutorsHolder executors = create_executors_fn();

  Executor::SetForegroundExecutor(executors.foreground_executor.get());
  Executor::SetBackgroundExecutor(executors.background_executor.get());

  return std::make_unique<ExecutorsHolder>(std::move(executors));
}

Future<absl::Status> DetachAndShutdownExecutors(
    std::unique_ptr<ExecutorsHolder> executors, bool use_async_shutdown) {
  assert(executors);

  if (Executor::BackgroundExecutor() == executors->background_executor.get()) {
    Executor::SetBackgroundExecutor(nullptr);
  }

  if (Executor::ForegroundExecutor() == executors->foreground_executor.get()) {
    Executor::SetForegroundExecutor(nullptr);
  }

  if (executors->foreground_executor) {
    executors->foreground_executor->Shutdown();
  }

  Future<absl::Status> result;

  if (use_async_shutdown) {
    auto cleanup_fn = [executors = std::move(executors),
                       weak_result = make_weak(result)]() {
      // Background executor needs to be shutdown on a separate thread to
      // prevent any remaining background work from causing the main thread to
      // stall during shutdown.
      if (executors->background_executor) {
        executors->background_executor->Shutdown();
      }

      if (absl::optional<Future<absl::Status>> result_opt =
              weak_result.Lock()) {
        return result_opt.value().Return(absl::OkStatus());
      }
    };

#if IMP_THREADS(GOOGLE3)
    StartDetachedThread("imp_cleanup", std::move(cleanup_fn));
#elif IMP_THREADS(STDLIB)
    std::thread cleanup(std::move(cleanup_fn));
    cleanup.detach();
#else
#error Invalid thread mode.
#endif
  } else {
    if (executors->background_executor) {
      executors->background_executor->Shutdown();
    }
  }

  return result;
}

}  // namespace imp
