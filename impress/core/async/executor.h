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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_EXECUTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_EXECUTOR_H_

#include <cstddef>
#include <functional>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "core/async/task.h"
#include "core/async/task_priority.h"
#include "core/common/invocable.h"

namespace imp {

// Interface class defining facilities for running arbitrary tasks, potentially
// in background threads.
//
// A task priority can be passed into Executor::Schedule or
// Executor::ScheduleInvocable to schedule tasks with a given priority, which is
// kNormalTaskPriority by default.
// Scheduling a task returns a TaskId representing the extant Executor task.
// This TaskId can be passed back into Executor::UpdateTaskPriority in order to
// update the priority of an extant Executor task.
//
// Although imp::Executor is similar to the thread::Executor from google3,
// Impress does not use thread::Executor for the following reasons:
// * //thread is not part of portable google3 and we need to run on non-prod
//   platforms including Android, iOS, and Windows.
// * We plan to open source, and //thread is not available outside of google3.
// * Since Filament uses its own JobSystem, we don't want to introduce a
//   dependency on //thread to applications that aren't already using it.
// * Impress uses the same Executor interface for creating tasks on the `main`
//   thread, which requires a `Pump` method (or similar) which is not part of
//   thread::Executor.
//
// For applications that already depend on //thread, it would be simple
// to make an imp::Executor that forwards to a thread::Executor, if desired.
//
// Example:
//   // Run DoSomething in the background.
//   Executor::BackgroundExecutor()->Schedule([]() { DoSomething(); });
class Executor {
 public:
  virtual ~Executor();

  // Type is used to describe the intended Executor.
  enum class Type {
    kImmediate,   // Blocks the scheduling thread and runs the tasks immediately
                  // when scheduled.
    kForeground,  // Schedules tasks to be run on the foreground
                  // thread when the executor is pumped, or whichever thread is
                  // meant to be the main executing one.
    kBackground,  // Schedules tasks to be run on a background
                  // thread.
    kCurrent      // Selects the active executor when the task is scheduled.
  };

  // Returns the string representation of Executor::Type.
  static std::string ToString(Type type);

  // Eventually runs the given function in a thread managed by this Executor.
  //
  // If task reprioritization is supported, returns a unique TaskId
  // representing the underlying executor Task.
  // If task scheduling is not supported, returns kGenericTaskId.
  // If the executor is shutting down, returns kInvalidTaskId.
  TaskId Schedule(std::function<void()> function,
                  int task_priority = kNormalTaskPriority);

  // Same as Schedule, but instead of taking a std::function, it takes an
  // imp::Invocable. The imp::Invocable is similar, but it is move-only and can
  // capture move only types.
  // This version also returns imp::kInvalidTaskId if an imp::Invocable that
  // does not contain a functor is passed in.
  virtual TaskId ScheduleInvocable(Invocable<void()> invocable,
                                   int task_priority = kNormalTaskPriority) = 0;

  // Returns a reserved TaskId, for which the corresponding functor and task
  // priority should be provided through a later call to
  // ScheduleWithReservedTaskId.
  //
  // Returns kGenericTaskId if task reprioritization is not supported on this
  // Executor.
  //
  // This function exists in order make Executor more compatible with async
  // code. Specifically, Impress Futures uses this API so that the TaskId of a
  // Future's task can be known before the task is scheduled.
  [[nodiscard]] virtual TaskId ReserveTaskId();
  // Schedules a task using a TaskId that MUST have been previously obtained
  // through a call to ReserveTaskId(). Use with caution.
  //
  // Returns false if the executor is shutting down.
  //
  // If reserved_task_id is equal to kGenericTaskId, this function behaves the
  // same as ScheduleInvocable and returns true if the task was scheduled.
  virtual bool ScheduleWithReservedTaskId(
      TaskId reserved_task_id, imp::Invocable<void()> invocable,
      int task_priority = kNormalTaskPriority);

  // Updates the priority of the task associated with the given TaskId.
  //
  // If reprioritization succeeds, if reprioritization is redundant, or if the
  // referenced task has already completed, returns absl::OkStatus().
  //
  // If the TaskId cannot resolve to any task, or if task reprioritization is
  // not supported on this Executor, returns an error.
  virtual absl::Status UpdateTaskPriority(TaskId task_id, int task_priority);

  // Tries to invoke the task associated with the given TaskId. Returns true if
  // the task was invoked, false otherwise.
  virtual bool InvokeScheduledTask(TaskId task_id);

  // Returns true if task reprioritization is supported on this executor.
  virtual bool IsTaskReprioritizingSupported() { return false; }

  // Returns the priority of the Task associated with the given TaskId.
  //
  // If the referenced task has completed, or if the TaskId cannot resolve to
  // any Task, or if updating priority is not supported on this Executor,
  // returns an error.
  virtual absl::StatusOr<int> GetTaskPriority(TaskId task_id);

  // Tells the Executor to shutdown.  When shutting down, executors will not
  // schedule new tasks.
  virtual void Shutdown() = 0;

  // Attempt to make progress on pending tasks.  Passing drain ==
  // true will attempt to complete all pending tasks, where drain
  // == false will attempt to complete a single pending task.
  // Intended to be called periodically on the main thread for the
  // ForegroundExecutor.
  // Returns whether any tasks were executed.
  virtual bool Pump(bool drain) { return false; }

  // Pumps until the queue is flushed, or until the timeout has elapsed.
  // Returns the number of tasks that were executed.
  virtual size_t DrainWithTimeout(absl::Duration timeout) { return 0; }

  // Returns true if this executor needs to be explicitly pumped to run tasks.
  // Otherwises, tasks are run automatically (i.e. in a thread pool).
  virtual bool IsPumpingRequired() { return true; }

  // Returns true if there are pending tasks waiting to be executed at the
  // next explicit pump/drain.
  virtual bool HasPendingTasks() { return false; }

  // Returns the number of pending tasks waiting to be executed at the
  // next explicit pump/drain.
  virtual int GetPendingTaskCount() { return -1; }

  // Gets the executor described by type.
  // If no executor is set for the type, nullptr is returned.
  //
  // Note: Except for kImmediate, which is always the same, Executors are
  // assigned by type per-thread.
  //
  // This makes it possible to have multiple threads with their own
  // "foreground executors. This also means that to access a background
  // thread's foreground executor, Executor::SetForegroundExecutor must be
  // called BOTH from the foreground thread and the background thread.
  static Executor* Get(Type type);

  // Gets the background executor associated with the current thread.
  static Executor* BackgroundExecutor();

  // Sets the background executor associated with the current thread.
  // Should be called from all threads that the background executor is going to
  // be accessed on, including both foreground and background threads.
  static void SetBackgroundExecutor(Executor* executor);

  // Gets the foreground executor associated with the current thread.
  static Executor* ForegroundExecutor();
  // Sets the foreground executor associated with the current thread.
  // Should be called from all threads that the background executor is going to
  // be accessed on, including both foreground and background threads.
  static void SetForegroundExecutor(Executor* executor);

  // Gets the executor that runs tasks on this thread.
  // This is typically either the foreground or background executor associated
  // with this thread. It is expected that there is only one executor
  // running tasks on a particular thread.
  static Executor* CurrentExecutor();

 protected:
  // This sets the executor that is running tasks on the current thread.
  //
  // Should be called by an Executor subclass on the thread that the executor
  // will execute tasks on while it is running.
  static void SetCurrentExecutor(Executor* executor);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_EXECUTOR_H_
