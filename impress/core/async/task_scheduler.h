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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_TASK_SCHEDULER_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_TASK_SCHEDULER_H_

#include <list>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "core/async/task.h"
#include "core/async/task_priority.h"
#include "core/common/invocable.h"
#include "core/common/robin_map.h"
#include "core/common/robin_set.h"
namespace imp {

// Default rate at which to increase the effective numeric Task priority by 1.
//
// With this default value of 100ms a Task scheduled 1 second ago with a numeric
// priority of 0 will have the same effective priority as a Task scheduled now
// with a task priority of 10.
constexpr absl::Duration kDefaultTaskAgeRate = absl::Milliseconds(100);

// A priority-based Task scheduler that supports rescheduling Tasks with
// updated priority values. TaskScheduler also boosts the priority of Tasks that
// have been scheduled earlier. See
// TaskSchedulerOptions::task_age_period_milliseconds.
//
// NOTE: This class is thread-unsafe. Every access must be locked if
// TaskScheduler is reachable from multiple threads. (broken link)
class TaskScheduler {
 public:
  struct TaskSchedulerOptions {
    // TaskScheduler boosts the priority of Tasks that have been scheduled
    // earlier. task_age_period_milliseconds is the period of time that is
    // weighted as equivalent to a difference in task priority of 1.
    //
    // This factor prevents Task starvation in cases where the rate of Tasks
    // being scheduled is close to the rate of Tasks being executed. In other
    // words, we are always limited by the throughput of the Executor and this
    // value just dictates the order of Task execution in cases where there is a
    // large backlog of Tasks.
    absl::Duration task_age_rate;
  };
  TaskScheduler(TaskSchedulerOptions options = {
                    .task_age_rate = kDefaultTaskAgeRate});

  // Push an Invocable to the TaskScheduler along with an associated priority.
  // Returns a TaskId that can be used to identify the Task. Returns a
  // InvalidArgumentError if the provided Invocable is invalid.
  absl::StatusOr<TaskId> PushTask(Invocable<void()> invocable,
                                  int priority = kNormalTaskPriority);

  // Same as PushTask, but uses a reserved TaskId. Returns an OkStatus if the
  // Task is successfully pushed. Returns an InvalidArgumentError if the
  // provided Invocable is invalid, or if the reserved TaskId has already been
  // used.
  absl::Status PushWithReservedTaskId(TaskId task_id,
                                      Invocable<void()> invocable,
                                      int priority);

  // Reserves a TaskId for later use in PushWithReservedTaskId(). It is
  // guaranteed that PushTask() will not return a TaskId that was reserved by
  // this method.
  [[nodiscard]] TaskId ReserveTaskId();

  // Pops and returns the Invocable of the next scheduled Task. If a TaskId is
  // provided, the Task with that TaskId is popped and returned. Returns a
  // FailedPreconditionError if there are no valid Tasks in the TaskScheduler,
  // or a NotFoundError if the provided TaskId does not resolve to a Task.
  [[nodiscard]] absl::StatusOr<Invocable<void()>> PopTask(
      std::optional<TaskId> task_id = std::nullopt);

  // Reschedules a Task with an updated task priority. Returns a NotFoundError
  // if the Task has already been completed, or if the TaskId does not resolve
  // to a Task.
  // Note: Internally, this does not reset the age of the Task, so it
  // is possible for a low priority Task to be scheduled first if it has
  // remained unscheduled for sufficiently long.
  absl::Status RescheduleTask(TaskId task_id, int task_priority);

  // Returns the priority of a Task pushed to this TaskScheduler. Returns a
  // NotFoundError if the Task has already been completed, or if the TaskId
  // does not resolve to a Task.
  absl::StatusOr<int> GetTaskPriority(TaskId task_id);

  // Returns the number of valid Tasks currently scheduled.
  int GetTaskCount() const;

  // Returns true if there are no valid Tasks in the TaskScheduler.
  bool IsEmpty() const;

  // Clears the TaskScheduler of all Tasks and reserved memory. After this
  // call, all extant TaskIds will become invalid and IsEmpty() will return
  // true.
  void Clear();

 private:
  // Pushes a Task to the TaskScheduler.
  void PushTaskInternal(Task* /*absl_nonnull*/  task);

  // Stores Tasks and allows random access to Tasks by TaskId. Tasks are
  // stored with RegisterTask(), assigned a unique TaskId, and moved out of the
  // TaskRegistry ReleaseTask().
  class TaskRegistry {
   public:
    // Constructs and stores a Task, returning a pointer to the Task, which
    // cannot be null. Returns InvalidArgumentError if a Task with the same
    // TaskId already exists, or if the provided Invocable is invalid.
    absl::StatusOr<Task* /*absl_nonnull*/ > RegisterTask(
        imp::Invocable<void()> invocable, int priority = kNormalTaskPriority,
        absl::Time creation_time = absl::Now(),
        std::optional<TaskId> reserved_task_id = std::nullopt);

    // Moves the Task out of the TaskRegistry and releases the memory back
    // to the TaskRegistry.
    std::unique_ptr<Task> ReleaseTask(TaskId task_id);

    // Returns the Task associated with the TaskId. If the TaskId cannot be
    // resolved, returns a NotFoundError.
    absl::StatusOr<Task* /*absl_nonnull*/ > GetTask(TaskId task_id);

    // Marks that original_task_id refers to the Task with TaskId
    // rescheduled_task_id. Further calls to GetTask(original_task_id) will
    // return the Task referenced by rescheduled_task_id.
    void SetOriginalAndRescheduledTask(TaskId original_task_id,
                                       TaskId rescheduled_task_id);

    // Clears the TaskRegistry of all Tasks and reserved memory. After this
    // call, all extant TaskIds will become invalid. This does not reset the
    // counter for the next TaskId.
    void Clear();

    // Marks a TaskId as reserved, returns it, and increments the next TaskId.
    int ReserveTaskId();

   private:
    // Pointer-stable Task storage, using TaskId::id as a key.
    imp::RobinMap<int, std::unique_ptr<Task>> task_map_;
    // For TaskIds A and B, if original_index_to_rescheduled_index_[A] = B
    // then the Task from GetTask(A) will return the same Task* as GetTask(B).
    imp::RobinMap<int, int> original_task_id_to_rescheduled_task_id_;
    // For TaskIds B and A, if rescheduled_index_to_original_index_[B] = A
    // then the Task from GetTask(A) will return the same Task* as GetTask(B).
    imp::RobinMap<int, int> rescheduled_task_id_to_original_task_id_;
    // A set of TaskIds that have been reserved by ReserveTaskId(). This is used
    // to ensure that PushWithReservedTaskId() can not be called twice, for
    // example.
    imp::RobinSet<int> reserved_task_ids_;
    // The underlying numeric id to assign to the next TaskId.
    int next_task_id_ = 0;
  };

  // A FIFO queue for all Tasks of the same priority.
  class TaskPriorityGroup {
   public:
    TaskPriorityGroup(int priority, Task* /*absl_nonnull*/  first_task)
        : priority_(priority), tasks_({first_task}) {};

    // Pushes a Task in FIFO order.
    void PushBack(Task* /*absl_nonnull*/  task);

    // Pops the next Task in FIFO order.
    void PopFront();

    // Returns the next Task in FIFO order.
    Task* /*absl_nonnull*/  Front();

    int GetPriority() const;
    bool IsEmpty() const;

   private:
    int priority_;
    // TODO Implement a ring buffer so we can use std::vector which
    // has a lesser impact on binary size.
    std::queue<Task*, std::list<Task*>> tasks_;
  };

  // Compares TaskPriorityGroups by looking at the highest priority Task within
  // each TaskPriorityGroup, accounting for Task age.
  struct TaskPriorityGroupLessThan {
    explicit TaskPriorityGroupLessThan(absl::Duration age_rate)
        : task_age_rate(age_rate) {}
    bool operator()(TaskPriorityGroup* group_1, TaskPriorityGroup* group_2);
    absl::Duration task_age_rate;
  };

  // Pops and returns the next Task, which may be marked as deleted. Returns a
  // FailedPreconditionError if there are no valid Tasks in the TaskScheduler.
  absl::StatusOr<Task* /*absl_nonnull*/ > PopCandidateTask();

  // At what rate to increase the effective Task priority by 1.
  absl::Duration task_age_rate_;

  // Provides random access to underlying Tasks, which are move-only.
  TaskRegistry task_registry_;

  // Provides random access to TaskPriorityGroups, using task priority as index.
  // The max size of this vector is kNumTaskPriorities.
  std::vector<std::unique_ptr<TaskPriorityGroup>>
      priority_to_task_priority_group_;

  // A priority queue of TaskPriorityGroups. Scales with the amount of distinct
  // priorities, with a maximum size of kNumTaskPriorities.
  std::priority_queue<TaskPriorityGroup*, std::vector<TaskPriorityGroup*>,
                      TaskPriorityGroupLessThan>
      task_priority_groups_ =
          std::priority_queue<TaskPriorityGroup*,
                              std::vector<TaskPriorityGroup*>,
                              TaskPriorityGroupLessThan>(
              TaskPriorityGroupLessThan(task_age_rate_));

  // A counter of how many Tasks are currently scheduled. This counter excludes
  // rescheduled Tasks - a call to RescheduleTask will not increment this
  // counter.
  int current_valid_tasks_ = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_TASK_SCHEDULER_H_
