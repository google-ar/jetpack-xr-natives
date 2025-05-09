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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_INFLIGHT_CREATION_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_INFLIGHT_CREATION_H_

#include <cstddef>
#include <cstdint>
#include <functional>

#include "absl/status/status.h"
#include "filament/filament/backend/include/backend/BufferDescriptor.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "core/common/invocable.h"
#include "core/image/image_contents.h"

namespace imp::loader::details {

// This structure is expected to have a stable pointer for the duration of GPU
// resource creation i.e. the duration from the first request made on the main
// thread to process a buffer (e.g. vertex buffer, texture buffer) like filling
// to the handling of the last request by filament's render thread. We use the
// callback in Filament's BufferDescriptor to do this.
class InflightCreation {
 public:
  absl::Status TryComplete();
  bool IsFullyLoaded() const;
  bool HasPendingWork() const;

  // Invokes the callback when IsFullyLoaded.
  // This should only be called after all MakeDescriptor and MakePixelDescriptor
  // calls have been issued.
  void WhenFullyLoaded(Invocable<void()> fn);
  void RemoveWhenFullyLoadedCallback();

  filament::backend::BufferDescriptor MakeDescriptor(void const* buffer,
                                                     size_t size);

  // Internal mechanism to call function set by WhenFullyLoaded.
  std::function<void()> CreateImageCallback();

  void FinishPostingResources();

 private:
  static void Callback(void* buffer, size_t size, void* user);

  // Do not call callback_() directly. Always call it via this function instead.
  // Post-condition: callback_ is unset.
  void SafeInvokeCallback();

  bool is_finished_posting_resources_ = false;
  int posted_resource_count_ = 0;
  int submitted_resource_count_ = 0;
  Invocable<void()> callback_;
};

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_INFLIGHT_CREATION_H_
