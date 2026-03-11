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

#include "core/common/rememberer.h"

#include <cassert>
#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/synchronization/mutex.h"
#include "core/async/executor.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"

namespace imp {

Rememberer::Rememberer() { info_ = std::make_shared<RemembererInfo>(); }

Rememberer::~Rememberer() {
  if (!info_) {
    return;
  }
  ClearRememberedInternal(true);
}

bool Rememberer::HasRemembered() const {
  absl::MutexLock lock(&info_->mu);
  return !info_->remembered_objects_map.empty();
}

void Rememberer::ClearRemembered() { ClearRememberedInternal(false); }

void Rememberer::ClearRememberedInternal(bool is_destroying_rememberer) {
  RememberedObjectsMap remembered_objects_map;
  {
    absl::MutexLock lock(&info_->mu);
    if (is_destroying_rememberer) {
      info_->can_remember_object = false;
    }
    std::swap(remembered_objects_map, info_->remembered_objects_map);
  }

  remembered_objects_map.clear();
}

Invocable<void()> Rememberer::Remember(Holdable holdable) {
  absl::MutexLock lock(&info_->mu);
  // If the Rememberer is being destructed, return an empty function and do not
  // remember.
  if (!info_->can_remember_object) {
    IMP_LOG(imp::WARNING) << "Attempting to call Rememberer::Remember while "
                    "Rememberer is being "
                    "destructed. Ignoring.";
    return []() {};
  }
  info_->remembered_objects_map.emplace(++info_->next_id, std::move(holdable));

  return GetForgetFunction(info_->next_id);
}

Invocable<void()> Rememberer::GetForgetFunction(const Id& id) {
  std::weak_ptr<RemembererInfo> weak_info = info_;
  return [weak_info, id]() {
    auto forget_fn = [weak_info, id]() {
      std::shared_ptr<RemembererInfo> info = weak_info.lock();
      if (!info) {
        return;
      }

      absl::MutexLock lock(&info->mu);
      info->remembered_objects_map.erase(id);
    };

    if (Executor::ForegroundExecutor() != nullptr &&
        Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
      Executor::ForegroundExecutor()->Schedule(std::move(forget_fn));
    } else {
      forget_fn();
    }
  };
}

}  // namespace imp
