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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASYNC_TASK_H_
#define THIRD_PARTY_IMPRESS_CORE_ASYNC_TASK_H_

#include <sys/stat.h>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <utility>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "core/async/task_priority.h"
#include "core/common/invocable.h"

namespace imp {

// A TaskId is an identifier used with the imp::Executor API to refer to a task
// that has been scheduled for execution.
//
// Typical usage is to store the TaskId returned from Executor::Schedule(...)
// and pass it back into Executor::UpdateTaskPriority(...) with an updated
// task priority.
//
// TaskIds are manually constructed by Executors, so they are not guaranteed by
// default to be unique. The same TaskId can refer to two different Tasks across
// two different imp::Executors, for example.
class TaskId {
 public:
  constexpr explicit TaskId(int id = -1) : id(id) {}
  bool operator==(const TaskId& other) const { return other.id == id; };
  bool operator!=(const TaskId& other) const { return !(other == *this); };

 private:
  friend struct std::hash<TaskId>;
  friend class TaskScheduler;
  // An ID used for TaskId equality.
  int id;
};

// Represents an invalid Task, or the absence of a Task.
constexpr TaskId kInvalidTaskId = TaskId();
// Represents a generic valid Task.
constexpr TaskId kGenericTaskId = TaskId(-2);

// A move-only wrapper for an imp::Invocable<void()> that also stores metadata
// such as priority and creation time. For use with Impress Executors.
class Task {
 public:
  Task()
      : invocable_(),
        invocable_valid_(false),
        id_(kInvalidTaskId),
        creation_time_(absl::Now()) {}
  Task(Invocable<void()> invocable, int priority, TaskId id,
       absl::Time creation_time = absl::Now())
      : invocable_(std::move(invocable)),
        invocable_valid_(true),
        priority_(
            std::clamp(priority, kMinimumTaskPriority, kMaximumTaskPriority)),
        id_(id),
        creation_time_(creation_time) {}

  // Returns the ID of this task, set manually during construction.
  TaskId GetId() const;

  absl::Time GetCreationTime() const;

  int GetPriority() const;

  // Moves the Invocable out of the Task.
  // After MoveInvocable() is called, the Task will be invalid.
  Invocable<void()> MoveInvocable();

  // Returns true if the invocable is valid.
  explicit operator bool() const noexcept;

 private:
  Task MoveWithPriority(int priority);
  Invocable<void()> invocable_;
  bool invocable_valid_;
  int priority_;
  TaskId id_;
  absl::Time creation_time_;
};
}  // namespace imp

// Allows TaskId to be used in robin maps.
namespace std {
template <>
struct hash<imp::TaskId> {
  size_t operator()(const imp::TaskId& id) const {
    return std::hash<int>()(id.id);
  }
};
}  // namespace std

#endif  // THIRD_PARTY_IMPRESS_CORE_ASYNC_TASK_H_
