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

#include "core/async/task.h"

#include <utility>

#include "absl/time/time.h"
#include "core/common/invocable.h"

namespace imp {

TaskId Task::GetId() const { return id_; }

int Task::GetPriority() const { return priority_; }

absl::Time Task::GetCreationTime() const { return creation_time_; }

Task::operator bool() const noexcept { return invocable_valid_; }

Invocable<void()> Task::MoveInvocable() {
  invocable_valid_ = false;
  return std::move(invocable_);
}

}  // namespace imp
