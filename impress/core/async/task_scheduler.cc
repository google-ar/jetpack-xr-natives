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

#include "core/async/task_scheduler.h"

#include <cmath>
#include <memory>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "core/async/task.h"
#include "core/async/task_priority.h"
#include "core/common/invocable.h"
#include "mediapipe/framework/port/status_macros.h"
namespace imp {

TaskScheduler::TaskScheduler(TaskSchedulerOptions options)
    : task_age_rate_(options.task_age_rate) {
  Clear();
}

bool TaskScheduler::TaskPriorityGroupLessThan::operator()(
    TaskScheduler::TaskPriorityGroup* group_1,
    TaskScheduler::TaskPriorityGroup* group_2) {
  Task* task_1 = group_1->Front();
  Task* task_2 = group_2->Front();
  int age_offset = floor(
      (task_1->GetCreationTime() - task_2->GetCreationTime()) / task_age_rate);
  return task_1->GetPriority() < (task_2->GetPriority() + age_offset);
}

absl::Nonnull<Task*> TaskScheduler::PopCandidateTask() {
  // Check the next TaskPriorityGroup on the priority queue.
  TaskPriorityGroup* task_priority_group = task_priority_groups_.top();

  // Resolve the highest priority task.
  Task* task_ptr = task_priority_group->Front();
  task_priority_group->PopFront();

  if (task_priority_group->IsEmpty()) {
    // If this was the last Task in the TaskPriorityGroup, erase it.
    task_priority_groups_.pop();
    priority_to_task_priority_group_[task_priority_group->GetPriority()] =
        nullptr;
  }
  return task_ptr;
}

void TaskScheduler::PushTaskInternal(absl::Nonnull<Task*> task) {
  current_valid_tasks_++;
  int task_priority = task->GetPriority();

  // If we already have a TaskPriorityGroup for this task priority, then push
  // the Task to that TaskPriorityGroup.
  if (priority_to_task_priority_group_[task_priority] != nullptr) {
    bool was_empty = priority_to_task_priority_group_[task_priority]->IsEmpty();
    priority_to_task_priority_group_[task_priority]->PushBack(task);
    if (!was_empty) {
      // Because there exists an earlier Task of the same priority, we don't
      // need to add this TaskPriorityGroup to the priority queue.
      return;
    }
    task_priority_groups_.push(
        priority_to_task_priority_group_[task_priority].get());
  } else {
    // If we are introducing a new task priority, then create a corresponding
    // TaskPriorityGroup.
    priority_to_task_priority_group_[task_priority] =
        std::make_unique<TaskPriorityGroup>(
            /*priority=*/task_priority,
            /*first_task=*/task);
    TaskPriorityGroup* task_priority_group =
        priority_to_task_priority_group_[task_priority].get();
    // Push the TaskPriorityGroup to the priority queue.
    task_priority_groups_.push(task_priority_group);
  }
}

std::unique_ptr<Task> TaskScheduler::PopTaskInternal() {
  if (IsEmpty()) {
    IMP_LOG(imp::FATAL) << "MoveNextTask() called with no tasks scheduled.";
  }
  current_valid_tasks_--;

  // Pop until we find the first valid Task.
  Task* next_task = nullptr;
  while (next_task == nullptr) {
    Task* candidate_task = PopCandidateTask();
    // Delete the task if it's invalid.
    if (!(*candidate_task)) {
      task_registry_.ReleaseTask(candidate_task->GetId());
    } else {
      next_task = candidate_task;
    }
  }

  // Return the underlying Task.
  return task_registry_.ReleaseTask(next_task->GetId());
}

absl::Status TaskScheduler::RescheduleTask(TaskId task_id, int task_priority) {
  MP_ASSIGN_OR_RETURN(Task * task_ptr, task_registry_.GetTask(task_id));

  // Early return if no changes are required.
  if (task_ptr->GetPriority() == task_priority) {
    return absl::OkStatus();
  }

  // A reschedule is a lazy deletion of the original Task, followed by pushing
  // a clone of the original Task with a higher priority value.
  //
  // First we move out the original imp::Invocable<void()> and assign it to a
  // new Task. This marks the original Task as invalid, meaning it still
  // exists in TaskRegistry and TaskPriorityGroup, but it will be deleted when
  // it next appears in PopCandidateTask().
  Invocable<void()> invocable = task_ptr->MoveInvocable();
  Task* new_task = task_registry_.RegisterTask(
      std::move(invocable), task_priority, task_ptr->GetCreationTime());
  PushTaskInternal(new_task);
  current_valid_tasks_--;

  // Create a reference between the original TaskId and the rescheduled TaskId.
  task_registry_.SetOriginalAndRescheduledTask(task_id, new_task->GetId());
  return absl::OkStatus();
}

absl::StatusOr<int> TaskScheduler::GetTaskPriority(TaskId task_id) {
  MP_ASSIGN_OR_RETURN(Task * task_ptr, task_registry_.GetTask(task_id));
  return task_ptr->GetPriority();
}

TaskId TaskScheduler::PushTask(Invocable<void()> invocable, int priority) {
  if (!invocable) {
    return kInvalidTaskId;
  }
  Task* task = task_registry_.RegisterTask(std::move(invocable), priority);
  PushTaskInternal(task);
  return task->GetId();
}

TaskId TaskScheduler::ReserveTaskId() {
  return TaskId(task_registry_.GetNextTaskId(/*increment=*/true));
}

void TaskScheduler::PushWithReservedTaskId(TaskId reserved_task_id,
                                           Invocable<void()> invocable,
                                           int priority) {
  // Same as a regular push, except we use the reserved TaskId instead of
  // picking a new one.
  Task* task = task_registry_.RegisterTask(std::move(invocable), priority,
                                           absl::Now(), reserved_task_id);
  PushTaskInternal(task);
}

Invocable<void()> TaskScheduler::PopTask() {
  return PopTaskInternal()->MoveInvocable();
}

bool TaskScheduler::IsEmpty() const { return current_valid_tasks_ == 0; }

void TaskScheduler::Clear() {
  // Create a map of task priority to TaskPriorityGroup.
  priority_to_task_priority_group_.clear();
  priority_to_task_priority_group_.reserve(kNumTaskPriorities);
  for (int i = 0; i < kNumTaskPriorities; ++i) {
    priority_to_task_priority_group_.push_back(nullptr);
  }
  task_registry_.Clear();
  task_priority_groups_ =
      std::priority_queue<TaskPriorityGroup*, std::vector<TaskPriorityGroup*>,
                          TaskPriorityGroupLessThan>(
          TaskPriorityGroupLessThan(task_age_rate_));
  current_valid_tasks_ = 0;
}

void TaskScheduler::TaskPriorityGroup::PushBack(absl::Nonnull<Task*> task) {
  tasks_.push(task);
}

void TaskScheduler::TaskPriorityGroup::PopFront() { tasks_.pop(); }

absl::Nonnull<Task*> TaskScheduler::TaskPriorityGroup::Front() {
  return tasks_.front();
}

int TaskScheduler::TaskPriorityGroup::GetPriority() const { return priority_; }

bool TaskScheduler::TaskPriorityGroup::IsEmpty() const {
  return tasks_.empty();
}

absl::Nonnull<Task*> TaskScheduler::TaskRegistry::RegisterTask(
    imp::Invocable<void()> invocable, int priority, absl::Time creation_time,
    std::optional<TaskId> reserved_task_id) {
  TaskId task_id = reserved_task_id.value_or(TaskId(next_task_id_++));
  auto result = task_map_.insert(
      {task_id.id, std::make_unique<Task>(std::move(invocable), priority,
                                          task_id, creation_time)});
  return result.first.value().get();
}

std::unique_ptr<Task> TaskScheduler::TaskRegistry::ReleaseTask(TaskId task_id) {
  // Find and remove the Task from the task map.
  auto task_iter = task_map_.find(task_id.id);
  if (task_iter == task_map_.end()) {
    return nullptr;
  }
  std::unique_ptr<Task> task = std::move(task_iter.value());
  task_map_.erase_fast(task_iter);
  // It's possible that the Task we are releasing is a rescheduled Task. In that
  // case, we need to delete the corresponding entry in
  // original_task_id_to_rescheduled_task_id_ so that this Task can no longer be
  // referenced.
  auto original_task_id_iter =
      rescheduled_task_id_to_original_task_id_.find(task_id.id);
  if (original_task_id_iter != rescheduled_task_id_to_original_task_id_.end()) {
    int original_task_id = original_task_id_iter->second;
    // Clean up the bidirectional reference between original TaskId and the
    // rescheduled TaskId.
    rescheduled_task_id_to_original_task_id_.erase(task_id.id);
    original_task_id_to_rescheduled_task_id_.erase(original_task_id);
  }

  return task;
}

absl::StatusOr<absl::Nonnull<Task*>> TaskScheduler::TaskRegistry::GetTask(
    TaskId task_id) {
  int resolved_task_id = task_id.id;
  // Attempt to resolve this TaskId to a rescheduled TaskId.
  auto rescheduled_task_id_iter =
      original_task_id_to_rescheduled_task_id_.find(task_id.id);
  if (rescheduled_task_id_iter !=
      original_task_id_to_rescheduled_task_id_.end()) {
    resolved_task_id = rescheduled_task_id_iter->second;
  }
  auto task_iter = task_map_.find(resolved_task_id);
  if (task_iter != task_map_.end()) {
    Task* task = task_iter->second.get();
    
    return task_iter->second.get();
  }
  return absl::NotFoundError(
      "Cannot resolve TaskId. Task may have been rescheduled or completed.");
}

void TaskScheduler::TaskRegistry::SetOriginalAndRescheduledTask(
    TaskId original_task_id, TaskId rescheduled_task_id) {
  // If the original_task_id has already been rescheduled, we want to first
  // clean up the existing references before setting new ones.
  auto old_rescheduled_task_id_iter =
      original_task_id_to_rescheduled_task_id_.find(original_task_id.id);
  if (old_rescheduled_task_id_iter !=
      original_task_id_to_rescheduled_task_id_.end()) {
    rescheduled_task_id_to_original_task_id_.erase(
        old_rescheduled_task_id_iter.value());
  }
  // Create a bidirectional reference between original TaskId and the
  // rescheduled TaskId.
  original_task_id_to_rescheduled_task_id_[original_task_id.id] =
      rescheduled_task_id.id;
  rescheduled_task_id_to_original_task_id_[rescheduled_task_id.id] =
      original_task_id.id;
}

void TaskScheduler::TaskRegistry::Clear() {
  task_map_.clear();
  rescheduled_task_id_to_original_task_id_.clear();
  original_task_id_to_rescheduled_task_id_.clear();
  // Do not clear next_task_id_.
}

int TaskScheduler::TaskRegistry::GetNextTaskId(bool increment) {
  if (increment) {
    return next_task_id_++;
  }
  return next_task_id_;
}

}  // namespace imp
