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

#include "core/async/executor.h"

#include <functional>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "core/async/task.h"
#include "core/async/task_priority.h"
#include "core/common/invocable.h"
#include "core/common/platform_helpers.h"
#include "core/common/string_helpers.h"

namespace imp {
namespace {

// Executor used to run a task immediately as opposed to scheduling for later.
// This executor should not be instantiated directly. The only instance should
// be the static pointer kept here and returned by a call to
// Executor::Get(kImmediate);
// N.B. CurrentExecutor() will NEVER return an ImmediateExecutor.
class ImmediateExecutor : public Executor {
 public:
  ImmediateExecutor() {}
  TaskId ScheduleInvocable(Invocable<void()> invocable,
                           int task_priority) override {
    invocable();
    // Since ImmediateExecutor runs tasks immediately and does not support task
    // rescheduling, we return an arbitrary valid TaskId here.
    return kGenericTaskId;
  }

  absl::Status UpdateTaskPriority(TaskId task_id, int task_priority) override {
    return absl::OkStatus();
  }

  // Immediate Executors cannot be shut down.
  void Shutdown() override {}
};

// Holds the background executor associated with each thread.
thread_local Executor* s_bg_executor = nullptr;

// Holds the foreground executor associated with each thread.
thread_local Executor* s_fg_executor = nullptr;

// Holds the currently running executor on each thread.
// On the foreground thread, this will be the same as the foreground executor.
// On a thread managed by a background executor, this should be the background
// executor.
thread_local Executor* s_current_executor = nullptr;

Executor* const s_im_executor = new ImmediateExecutor();

}  // namespace

Executor::~Executor() {
  if (BackgroundExecutor() == this) {
    SetBackgroundExecutor(nullptr);
  }

  if (ForegroundExecutor() == this) {
    SetForegroundExecutor(nullptr);
  }
}

TaskId Executor::Schedule(std::function<void()> function, int task_priority) {
  return ScheduleInvocable(std::move(function), task_priority);
}

TaskId Executor::ReserveTaskId() { return kGenericTaskId; }

bool Executor::ScheduleWithReservedTaskId(TaskId reserved_task_id,
                                          imp::Invocable<void()> invocable,
                                          int task_priority) {
  if (reserved_task_id == kGenericTaskId) {
    return ScheduleInvocable(std::move(invocable), task_priority) !=
           kInvalidTaskId;
  }
  return false;
}

absl::Status Executor::UpdateTaskPriority(TaskId task_id, int task_priority) {
  return absl::UnimplementedError(
      "UpdateTaskPriority is not supported on this Executor.");
};

bool Executor::InvokeScheduledTask(TaskId task_id) {
  // By default, executors do not support invoking scheduled tasks.
  return false;
};

absl::StatusOr<int> Executor::GetTaskPriority(TaskId task_id) {
  return absl::UnimplementedError(
      "GetTaskPriority is not supported on this Executor.");
}

std::string Executor::ToString(Type type) {
  switch (type) {
    case Type::kImmediate:
      return "kImmediate";
    case Type::kForeground:
      return "kForeground";
    case Type::kBackground:
      return "kBackground";
    case Type::kCurrent:
      return "kCurrent";
  }
}

Executor* Executor::Get(Type type) {
  Executor* ex = nullptr;
  switch (type) {
    case Type::kImmediate:
      ex = s_im_executor;
      break;
    case Type::kForeground: {
      ex = s_fg_executor;
    } break;
    case Type::kBackground: {
      ex = s_bg_executor;
    } break;
    case Type::kCurrent:
      ex = s_current_executor;
      break;
  }

  return ex;
}

Executor* Executor::BackgroundExecutor() { return s_bg_executor; }

void Executor::SetBackgroundExecutor(Executor* executor) {
  s_bg_executor = executor;
}
Executor* Executor::ForegroundExecutor() { return s_fg_executor; }

void Executor::SetForegroundExecutor(Executor* executor) {
  s_fg_executor = executor;
}

Executor* Executor::CurrentExecutor() { return s_current_executor; }

bool Executor::IsOnForegroundExecutor() {
  Executor* fg = ForegroundExecutor();
  return fg != nullptr && CurrentExecutor() == fg;
}

void Executor::SetCurrentExecutor(Executor* executor) {
  if (executor == s_im_executor) {
    IMP_LOG(imp::FATAL) << "cannot set the current executor to "
               << ToString(Type::kImmediate);
  }
  s_current_executor = executor;
}

}  // namespace imp
