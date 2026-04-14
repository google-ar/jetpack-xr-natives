/*
 * Copyright 2026 Google LLC
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

#include "core/async/executor_flags.h"

#include "absl/base/no_destructor.h"
#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"

namespace imp {
namespace {
absl::NoDestructor<absl::Mutex> g_executor_flags_mu;
bool g_should_simple_executor_destroy_tasks_on_shutdown
    ABSL_GUARDED_BY(*g_executor_flags_mu) = false;
}  // namespace

void ExecutorFlags::EnableSimpleExecutorToDestroyTasksOnShutdown() {
  absl::MutexLock lock(*g_executor_flags_mu);
  g_should_simple_executor_destroy_tasks_on_shutdown = true;
}

bool ExecutorFlags::ShouldSimpleExecutorDestroyTasksOnShutdown() {
  absl::MutexLock lock(*g_executor_flags_mu);
  return g_should_simple_executor_destroy_tasks_on_shutdown;
}

}  // namespace imp
