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

#include "core/async/simple_executor.h"

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "core/async/executor.h"
#include "core/async/task.h"
#include "core/async/task_scheduler.h"
#include "core/common/invocable.h"

namespace imp {

SimpleExecutor::SimpleExecutor(std::function<void(Executor*)> init_fn,
                               std::function<void(Executor*)> join_fn)
    : join_fn_(std::move(join_fn)),
      task_scheduler_(std::make_unique<TaskScheduler>()) {
  if (init_fn) {
    init_fn(this);
  }
}

SimpleExecutor::~SimpleExecutor() { Shutdown(); }

TaskId SimpleExecutor::ScheduleInvocable(Invocable<void()> invocable,
                                         int task_priority) {
  absl::MutexLock lock(mu_);
  if (finished_) {
    return kInvalidTaskId;
  }
  const absl::StatusOr<TaskId> status_or_task_id =
      task_scheduler_->PushTask(std::move(invocable), task_priority);
  if (!status_or_task_id.ok()) {
    return kInvalidTaskId;
  }
  return *status_or_task_id;
}

TaskId SimpleExecutor::ReserveTaskId() {
  absl::MutexLock lock(mu_);
  if (finished_) {
    return kInvalidTaskId;
  }
  return task_scheduler_->ReserveTaskId();
}

bool SimpleExecutor::ScheduleWithReservedTaskId(TaskId reserved_task_id,
                                                imp::Invocable<void()> function,
                                                int task_priority) {
  absl::MutexLock lock(mu_);
  if (finished_) {
    return false;
  }
  absl::Status push_status = task_scheduler_->PushWithReservedTaskId(
      reserved_task_id, std::move(function), task_priority);
  if (!push_status.ok()) {
    return false;
  }
  return true;
}

absl::Status SimpleExecutor::UpdateTaskPriority(TaskId task_id,
                                                int task_priority) {
  absl::MutexLock lock(mu_);
  if (finished_) {
    return absl::OkStatus();
  }
  return task_scheduler_->RescheduleTask(task_id, task_priority);
}

bool SimpleExecutor::InvokeScheduledTask(TaskId task_id) {
  absl::StatusOr<Invocable<void()>> invocable;
  {
    absl::MutexLock lock(mu_);
    if (finished_) {
      return false;
    }
    invocable = task_scheduler_->PopTask(task_id);
    if (!invocable.ok()) {
      return false;
    }
  }
  (*invocable)();
  return true;
}

absl::StatusOr<int> SimpleExecutor::GetTaskPriority(TaskId task_id) {
  absl::MutexLock lock(mu_);
  if (finished_) {
    return absl::FailedPreconditionError("Executor is shutdown.");
  }
  return task_scheduler_->GetTaskPriority(task_id);
}

void SimpleExecutor::Shutdown() {
  {
    absl::MutexLock lock(mu_);
    if (finished_) {
      return;
    }
    finished_ = true;
  }
  if (join_fn_) {
    join_fn_(this);
    join_fn_ = nullptr;
  }
}

size_t SimpleExecutor::PumpInternal(bool drain) {
  // Ensure that the current executor is set correctly while running the
  // scheduled tasks.
  Executor* previous_current_executor = Executor::CurrentExecutor();
  Executor::SetCurrentExecutor(this);

  std::vector<Invocable<void()>> invocables;
  while (!task_scheduler_->IsEmpty()) {
    absl::StatusOr<Invocable<void()>> status_or_invocable =
        task_scheduler_->PopTask();
    if (!status_or_invocable.ok()) {
      break;
    }
    invocables.push_back(*std::move(status_or_invocable));
    if (!drain) {
      break;
    }
  }

  // It's expected that mu_ is locked prior to calling PumpInternal().
  mu_.unlock();
  size_t invocables_size = invocables.size();
  for (auto& invocable : invocables) {
    invocable();
  }
  // We want to release any shared pointer ownership here before locking the
  // Executor again.
  invocables.clear();
  mu_.lock();

  Executor::SetCurrentExecutor(previous_current_executor);

  return invocables_size;
}

bool SimpleExecutor::Pump(bool drain) {
  absl::MutexLock lock(mu_);
  return PumpInternal(drain) > 0;
}

bool SimpleExecutor::HasPendingTasks() {
  absl::MutexLock lock(mu_);
  if (finished_) {
    return false;
  }
  return !task_scheduler_->IsEmpty();
}

int SimpleExecutor::GetPendingTaskCount() {
  absl::MutexLock lock(mu_);
  if (finished_) {
    return 0;
  }
  return task_scheduler_->GetTaskCount();
}

size_t SimpleExecutor::DrainWithTimeout(absl::Duration timeout) {
  size_t sum = 0, step = 0;
  auto start = absl::Now();
  absl::MutexLock lock(mu_);
  do {
    step = PumpInternal(/*drain = */ false);
    sum += step;
  } while (step && absl::Now() - start < timeout);
  return sum;
}

void SimpleExecutor::PumpLoop() {
  Executor::SetCurrentExecutor(this);
  // Define a condition for when there is work to do or we're finished.
  auto wait_for_task_or_finish = [this] {
    mu_.AssertHeld();
    return !task_scheduler_->IsEmpty() || finished_;
  };
  absl::MutexLock l(mu_);
  while (!finished_) {
    // Do a little work at a time.
    PumpInternal(false);
    // Unlock and Block this thread until there's work to do or we're finished.
    mu_.Await(absl::Condition(&wait_for_task_or_finish));
  }
  PumpInternal(true);
}

SimpleForegroundExecutor::SimpleForegroundExecutor()
    : SimpleExecutor([](Executor* ex) { SetCurrentExecutor(ex); },
                     [](Executor*) { SetCurrentExecutor(nullptr); }) {}

}  // namespace imp
