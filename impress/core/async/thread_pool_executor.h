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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_THREAD_POOL_EXECUTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_THREAD_POOL_EXECUTOR_H_

#include <condition_variable>  // NOLINT(build/c++11)
#include <memory>
#include <mutex>  // NOLINT(build/c++11)
#include <optional>
#include <thread>  // NOLINT(build/c++11)
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/optional.h"
#include "core/async/executor.h"
#include "core/async/task.h"
#include "core/async/task_scheduler.h"
#include "core/common/invocable.h"
#include "core/config.h"

#if IMP_THREADS(GOOGLE3)
#include "thread/thread.h"
#endif

namespace imp {

class ThreadPoolExecutor : public Executor {
 public:
  ThreadPoolExecutor(Executor* foreground_executor);
  ~ThreadPoolExecutor() override;

  TaskId ScheduleInvocable(Invocable<void()> invocable,
                           int task_priority = kNormalTaskPriority) override;

  TaskId ReserveTaskId() override;

  bool ScheduleWithReservedTaskId(TaskId reserved_task_id,
                                  imp::Invocable<void()> function,
                                  int task_priority) override;

  absl::Status UpdateTaskPriority(TaskId task_id, int priority) override;

  absl::StatusOr<int> GetTaskPriority(TaskId task_id) override;

  bool IsTaskReprioritizingSupported() override { return true; }

  void Shutdown() override;

  // Attempts to make progress on pending tasks.
  // Must be called from outside one of the ThreadPoolExecutor's threads.
  // If drain==true, then blocks until all pending tasks are completed.
  // Otherwise, returns false and does nothing.
  bool Pump(bool drain) override;

  // ThreadPoolExecutor does not need to be pumped, worker threads
  // automatically run tasks.
  bool IsPumpingRequired() override;

 private:
#if IMP_THREADS(GOOGLE3)
  class WorkerThread : public Thread {
   public:
    explicit WorkerThread(ThreadPoolExecutor* thread_pool_executor,
                          Executor* foreground_executor);

   protected:
    void Run() override;

   private:
    ThreadPoolExecutor* thread_pool_executor_;
    Executor* foreground_executor_;
  };
#endif

  std::optional<Invocable<void()>> WaitForTask();
  bool ProcessNextRequest();

#if IMP_THREADS(GOOGLE3)
  std::vector<std::unique_ptr<WorkerThread>> worker_threads_;
#elif IMP_THREADS(STDLIB)
  std::vector<std::thread> worker_threads_;
#else
#error Invalid thread mode.
#endif

  void WaitUntilDrained();

  absl::Mutex mu_;
  absl::CondVar condvar_;
  absl::CondVar wait_condvar_;

  bool finished_ ABSL_GUARDED_BY(mu_);
  int callback_counter_ ABSL_GUARDED_BY(mu_);
  std::unique_ptr<TaskScheduler> task_scheduler_ ABSL_GUARDED_BY(mu_);
};

using ThreadPoolThreadBegin = imp::Invocable<void()>;
using ThreadPoolThreadEnd = imp::Invocable<void()>;

// Special hook into the internals of ThreadPoolExecutor for tracing
// performance. Do not use unless you have a good reason.
void SetCallbacksForThreadPool(ThreadPoolThreadBegin thread_begin,
                               ThreadPoolThreadEnd thread_end);
void ClearCallbacksForThreadPool();

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_THREAD_POOL_EXECUTOR_H_
