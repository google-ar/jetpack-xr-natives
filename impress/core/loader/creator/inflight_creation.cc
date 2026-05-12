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

#include "core/loader/creator/inflight_creation.h"

#include <cstddef>
#include <functional>
#include <utility>

#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "filament/filament/backend/include/backend/BufferDescriptor.h"
#include "core/common/invocable.h"

namespace imp::loader::details {

bool InflightCreation::IsFullyLoaded() const {
  absl::MutexLock lock(mutex_);
  return posted_resource_count_ == submitted_resource_count_ &&
         is_finished_posting_resources_;
}

bool InflightCreation::HasPendingWork() const {
  absl::MutexLock lock(mutex_);
  return posted_resource_count_ != submitted_resource_count_;
}

// TODO This pattern is useful for all loaders; make it common?
void InflightCreation::WhenFullyLoaded(Invocable<void()> fn) {
  callback_ = std::move(fn);
  if (IsFullyLoaded()) {
    SafeInvokeCallback();
  }
}

void InflightCreation::RemoveWhenFullyLoadedCallback() { callback_ = {}; }

absl::Status InflightCreation::TryComplete() {
  if (IsFullyLoaded()) {
    SafeInvokeCallback();
    return absl::OkStatus();
  } else {
    return absl::UnavailableError("Not complete");
  }
}

void InflightCreation::FinishPostingResources() {
  {
    absl::MutexLock lock(mutex_);
    is_finished_posting_resources_ = true;
  }
  (void)TryComplete();
}

filament::backend::BufferDescriptor InflightCreation::MakeDescriptor(
    void const* buffer, size_t size) {
  {
    absl::MutexLock lock(mutex_);
    ++posted_resource_count_;
  }
  return filament::backend::BufferDescriptor(buffer, size,
                                             InflightCreation::Callback, this);
}

std::function<void()> InflightCreation::CreateImageCallback() {
  {
    absl::MutexLock lock(mutex_);
    ++posted_resource_count_;
  }
  return [this]() { Callback(nullptr, 0, this); };
}

void InflightCreation::SafeInvokeCallback() {
  // `this` must not be touched after invoking the registered callback because
  // the callback is allowed to delete the calling InflightCreation object.
  if (callback_) {
    Invocable<void()> callback = std::move(callback_);
    callback();
  }
}

// static
void InflightCreation::Callback(void* buffer, size_t size, void* user) {
  auto* self = reinterpret_cast<InflightCreation*>(user);
  {
    absl::MutexLock lock(self->mutex_);
    ++self->submitted_resource_count_;
  }
  if (self->IsFullyLoaded()) {
    self->SafeInvokeCallback();
  }
}

}  // namespace imp::loader::details
