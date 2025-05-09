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

#include "core/async/future_group.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "core/async/future_impl.h"

namespace imp {

void FutureGroup::UpdatePriority(int priority) {
  absl::MutexLock lock(&impl_->mutex);
  impl_->cached_priority = priority;
  // Update the priority of all Futures, cleaning up expired futures as we go.
  std::vector<std::weak_ptr<internal::FutureImpl>> future_group_elements;
  for (auto& future : impl_->future_group_elements_) {
    if (auto locked_future = future.second.lock()) {
      locked_future->UpdatePriority(priority);
    }
  }
}

void FutureGroup::AddFuture(std::shared_ptr<internal::FutureImpl> future) {
  // std::weak_ptr is not hashable so use an ID.
  int future_group_element_id = 0;
  {
    absl::MutexLock lock(&impl_->mutex);
    future_group_element_id = impl_->next_future_group_element_id++;
    impl_->future_group_elements_.insert(
        {future_group_element_id, std::weak_ptr<internal::FutureImpl>(future)});
  }
  future->UpdatePriority(GetTaskPriority());
  future->OnReady([impl = std::weak_ptr<FutureGroup::FutureGroupImpl>(impl_),
                   future_group_element_id = future_group_element_id](
                      internal::ResultHolder& result) {
    if (auto locked_impl = impl.lock()) {
      absl::MutexLock lock(&locked_impl->mutex);
      locked_impl->future_group_elements_.erase(future_group_element_id);
    }
  });
}

int FutureGroup::GetTaskPriority() const {
  absl::MutexLock lock(&impl_->mutex);
  return impl_->cached_priority;
}

}  // namespace imp
