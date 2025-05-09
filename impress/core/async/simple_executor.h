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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_SIMPLE_EXECUTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_SIMPLE_EXECUTOR_H_

#include <cstddef>
#include <functional>
#include <memory>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "core/async/executor.h"
#include "core/async/task.h"
#include "core/async/task_priority.h"
#include "core/async/task_scheduler.h"
#include "core/common/invocable.h"

namespace imp {

// Simple implementation of the Executor interface.  Spawning and joining of
// any worker threads is parameterized by functions passed to the constructor.
// This allows a simple, common interface to be usable by both a foreground
// executor (no background threads, Pump called explicitly) and a background
// executor that invokes PumpLoop in one or more worker threads.
class SimpleExecutor : public Executor {
 public:
  // Creates a new SimpleExecutor.  The init_fn is responsible for any
  // 'startup' work (i.e. spawning background threads which call
  // PumpLoop).  The join_fn is invoked when the executor is shutting
  // down, and is responsible for cleanup (i.e. joining spawned
  // background threads).
  explicit SimpleExecutor(
      std::function<void(Executor*)> init_fn = std::function<void(Executor*)>(),
      std::function<void(Executor*)> join_fn =
          std::function<void(Executor*)>());

  ~SimpleExecutor() override;

  TaskId ScheduleInvocable(Invocable<void()> invocable,
                           int task_priority = kNormalTaskPriority) override;

  TaskId ReserveTaskId() override;

  bool ScheduleWithReservedTaskId(TaskId reserved_task_id,
                                  imp::Invocable<void()> function,
                                  int task_priority) override;

  absl::Status UpdateTaskPriority(TaskId task_id, int task_priority) override;

  absl::StatusOr<int> GetTaskPriority(TaskId task_id) override;

  bool IsTaskReprioritizingSupported() override { return true; }

  void Shutdown() override;

  bool Pump(bool drain) override;

  bool HasPendingTasks() override;

  // Returns the number of tasks performed
  size_t DrainWithTimeout(absl::Duration timeout) override;

  // Run Pump continuously for the lifetime of the Executor.  This would
  // normally be called from a background thread managed by the executor.
  virtual void PumpLoop();

 private:
  // Returns the number of tasks executed.
  size_t PumpInternal(bool drain) ABSL_EXCLUSIVE_LOCKS_REQUIRED(mu_);

  std::function<void(Executor*)> join_fn_;
  absl::Mutex mu_;
  bool finished_ ABSL_GUARDED_BY(mu_) = false;
  std::unique_ptr<TaskScheduler> task_scheduler_ ABSL_GUARDED_BY(mu_);
};

// SimpleExecutor configured to act as a ForegroundExecutor.
class SimpleForegroundExecutor : public SimpleExecutor {
 public:
  SimpleForegroundExecutor();
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_SIMPLE_EXECUTOR_H_
