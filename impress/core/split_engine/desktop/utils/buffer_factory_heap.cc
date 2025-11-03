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

#include "core/split_engine/desktop/utils/buffer_factory_heap.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <new>
#include <utility>

#include "absl/base/nullability.h"
#include "absl/log/check.h"
#include "absl/numeric/int128.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "core/split_engine/desktop/utils/buffer_factory.h"

namespace imp::split_engine {
namespace {
// Buffer that owns the memory allocated from heap.
//
// The memory is allocated using `new (std::nothrow)` in
// BufferFactory::CreateBuffer and stored in Buffer's std::unique_ptr.
//
// The memory is freed when the buffer is destroyed.
class HeapBuffer : public Buffer {
 public:
  using ReleaseFunction = std::function<void()>;

  // HeapBuffer takes ownership of the memory allocated by the factory.
  // ReleaseFunction is called in ~HeapBuffer.
  HeapBuffer(/*absl_nonnull*/  std::unique_ptr<uint8_t[]> data, size_t size_in_bytes,
             ReleaseFunction&& release_function)
      : size_in_bytes_(size_in_bytes),
        release_function_(std::move(release_function)),
        data_(std::move(data)) {}
  ~HeapBuffer() override { release_function_(); };
  HeapBuffer(const HeapBuffer&) = delete;
  HeapBuffer& operator=(const HeapBuffer&) = delete;
  HeapBuffer(HeapBuffer&&) = delete;
  HeapBuffer& operator=(HeapBuffer&&) = delete;

  uint8_t* /*absl_nonnull*/  Data() noexcept override { return data_.get(); }
  const uint8_t* /*absl_nonnull*/  Data() const noexcept override {
    return data_.get();
  }
  size_t Size() const noexcept override { return size_in_bytes_; }

 private:
  const size_t size_in_bytes_;
  const ReleaseFunction release_function_;
  std::unique_ptr<uint8_t[]> data_;
};

}  // namespace

HeapBufferFactory::HeapBufferFactory(size_t quota_bytes)
    : quota_bytes_(quota_bytes) {}

absl::StatusOr</*absl_nonnull*/  std::unique_ptr<Buffer>>
HeapBufferFactory::CreateBuffer(size_t size_in_bytes) noexcept {
  if (size_in_bytes == 0) {
    return absl::InvalidArgumentError("Buffer size must be positive.");
  }

  uint8_t* data = nullptr;
  {
    absl::MutexLock lock(mutex_);
    // Enforce the quota.
    const absl::uint128 used_bytes = used_bytes_;
    const absl::uint128 size_bytes = size_in_bytes;
    if (used_bytes + size_bytes > quota_bytes_) {
      return absl::ResourceExhaustedError("Memory quota exceeded.");
    }

    // Use `new (std::nothrow)` to avoid crashing the app if the system runs out
    // of memory.
    data = new (std::nothrow) uint8_t[size_in_bytes];
    if (data == nullptr) {
      return absl::InternalError("Failed to allocate memory.");
    }

    // Update the used bytes.
    used_bytes_ += size_in_bytes;
  }

  return std::make_unique<HeapBuffer>(std::unique_ptr<uint8_t[]>(data),
                                      size_in_bytes, [this, size_in_bytes]() {
                                        // Decrement the used bytes when
                                        // HeapBuffer is destroyed.
                                        absl::MutexLock lock(mutex_);
                                        
                                        used_bytes_ -= size_in_bytes;
                                      });
}

}  // namespace imp::split_engine
