// Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_BUFFER_FACTORY_HEAP_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_BUFFER_FACTORY_HEAP_H_

#include <cstddef>
#include <memory>

#include "absl/base/nullability.h"
#include "absl/base/thread_annotations.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "core/split_engine/desktop/utils/buffer_factory.h"

namespace imp::split_engine {

// A buffer factory with memory allocation quota that uses heap to allocate
// memory. If the quota is exceeded, the factory will return error status until
// some buffer is freed.
class HeapBufferFactory : public BufferFactory {
 public:
  explicit HeapBufferFactory(size_t quota_bytes);

  // Creates a buffer of the given size. Returns an error if the buffer
  // could not be created (either allocation failed or quota was exceeded).
  absl::StatusOr</*absl_nonnull*/ std::unique_ptr<Buffer>> CreateBuffer(
      size_t size_in_bytes) noexcept override;

 private:
  // The maximum amount of bytes that can be allocated by this factory.
  const size_t quota_bytes_;

  absl::Mutex mutex_;
  // The amount of bytes that have been used from the quota.
  size_t used_bytes_ ABSL_GUARDED_BY(mutex_) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_BUFFER_FACTORY_HEAP_H_
