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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_EXECUTOR_PROVIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_EXECUTOR_PROVIDER_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/invocable.h"

namespace imp {

// Helper struct for holding a foreground and background executor in memory.
struct ExecutorsHolder {
  std::unique_ptr<Executor> foreground_executor;
  std::unique_ptr<Executor> background_executor;
};

// Functor passed into TryCreateAndSetExecutors to instantiate the executors if
// they can be set.
using CreateExecutorsFn = Invocable<ExecutorsHolder()>;

// Tries to create executors using the functors passed in.
//
// The executors are only created no foreground or background executors are
// currently associated with this thread.
//
// An AlreadyExistsError is returned if Executor::ForegroundExecutor or
// Executor::BackgroundExecutor has already been assigned on this thread,
// since there can only be one per-thread.
absl::StatusOr<std::unique_ptr<ExecutorsHolder>> TryCreateAndSetExecutors(
    CreateExecutorsFn create_executors_fn);

// Spawns a new detached cleanup thread and shuts down first the foreground
// executor, and then the background executor on the cleanup thread. This can be
// used to easily shut down the executors without blocking the foreground
// thread.
//
// The executors are kept in memory until after both executors have finished
// shutting down.
//
// The returned future becomes ready when shutdown has completed.
Future<absl::Status> DetachAndShutdownExecutors(
    std::unique_ptr<ExecutorsHolder> executors, bool use_async_shutdown = true);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_EXECUTOR_PROVIDER_H_
