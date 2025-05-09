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
namespace imp {

// Default rate at which to increase the effective numeric Task priority by 1.
//
// With this default value of 100ms a Task scheduled 1 second ago with a numeric
// priority of 0 will have the same effective priority as a Task scheduled now
// with a task priority of 10.
constexpr absl::Duration kDefaultTaskAgeRate = absl::Milliseconds(100);

/*
 * A priority-based Task scheduler that supports rescheduling Tasks with
 * updated priority values. TaskScheduler also boosts the priority of Tasks that
 * have been scheduled earlier. See
 * TaskSchedulerOptions::task_age_period_milliseconds.
 *
 * NOTE: This class is thread-unsafe. Every access must be locked if
 * TaskScheduler is reachable from multiple threads. (broken link)
 */
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
  // Returns a TaskId representing the extant task.
  // If the Invocable does not contain a functor, returns kInvalidTaskId.
  TaskId PushTask(Invocable<void()> invocable,
                  int priority = kNormalTaskPriority);

  [[nodiscard]] TaskId ReserveTaskId();
  void PushWithReservedTaskId(TaskId task_id, Invocable<void()> invocable,
                              int priority);

  // Pops and returns the Invocable of the next scheduled Task.
  [[nodiscard]] Invocable<void()> PopTask();

  // Reschedules a Task with an updated Priority. Returns a kNotFound error if
  // the Task has already been completed, or if the TaskId does not resolve to a
  // Task. Note: Internally, this does not reset the age of the Task, so it is
  // possible for a low priority Task to be scheduled first if it has remained
  // unscheduled for sufficiently long.
  absl::Status RescheduleTask(TaskId task_id, int task_priority);

  // Returns the priority of a Task pushed to this TaskScheduler. Returns a
  // kNotFound error if the Task has already been completed, or if the TaskId
  // does not resolve to a Task.
  absl::StatusOr<int> GetTaskPriority(TaskId task_id);

  // Returns true if there are no Tasks in the TaskScheduler.
  bool IsEmpty() const;

  // Clears all Tasks in the TaskScheduler.
  void Clear();

 private:
  // Pushes a Task to the TaskScheduler.
  void PushTaskInternal(absl::Nonnull<Task*> task);
  // Pops the next Task to be scheduled.
  std::unique_ptr<Task> PopTaskInternal();

  // Stores Tasks and allows random access to Tasks by TaskId. Tasks are
  // stored with RegisterTask(), assigned a unique TaskId, and moved out of the
  // TaskRegistry ReleaseTask().
  class TaskRegistry {
   public:
    // Stores a Task and returns a TaskId representing the extant Task.
    absl::Nonnull<Task*> RegisterTask(
        imp::Invocable<void()> invocable, int priority = kNormalTaskPriority,
        absl::Time creation_time = absl::Now(),
        std::optional<TaskId> reserved_task_id = std::nullopt);
    // Moves the Task out of the TaskRegistry and releases the memory back
    // to the TaskRegistry.
    std::unique_ptr<Task> ReleaseTask(TaskId task_id);
    // Returns the Task associated with the TaskId. If the TaskId cannot be
    // resolved, returns a kNotFound error.
    absl::StatusOr<absl::Nonnull<Task*>> GetTask(TaskId task_id);
    // Marks that original_task_id refers to the Task with TaskId
    // rescheduled_task_id. Further calls to GetTask(original_task_id) will
    // return the Task referenced by rescheduled_task_id.
    void SetOriginalAndRescheduledTask(TaskId original_task_id,
                                       TaskId rescheduled_task_id);
    void Clear();
    int GetNextTaskId(bool increment = false);

   private:
    // Pointer-stable Task storage, using TaskId::id as a key.
    imp::RobinMap<int, std::unique_ptr<Task>> task_map_;
    // For TaskIds A and B, if original_index_to_rescheduled_index_[A] = B
    // then the Task from GetTask(A) will be the same as the Task from
    // GetTask(B).
    imp::RobinMap<int, int> original_task_id_to_rescheduled_task_id_;
    // For TaskIds B and A, if rescheduled_index_to_original_index_[B] = A
    // then the Task from GetTask(A) will be the same as the Task from
    // GetTask(B).
    imp::RobinMap<int, int> rescheduled_task_id_to_original_task_id_;
    // The underlying numeric id to assign to the next TaskId.
    int next_task_id_ = 0;
  };

  // A FIFO queue for all Tasks of the same priority.
  class TaskPriorityGroup {
   public:
    TaskPriorityGroup(int priority, absl::Nonnull<Task*> first_task)
        : priority_(priority), tasks_({first_task}) {};

    // Pushes a Task in FIFO order.
    void PushBack(absl::Nonnull<Task*> task);

    // Pops the next Task in FIFO order.
    void PopFront();

    // Returns the next Task in FIFO order.
    absl::Nonnull<Task*> Front();

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

  // Pops and returns the next Task, which may be marked as deleted.
  absl::Nonnull<Task*> PopCandidateTask();

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
