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

#include "core/split_engine/desktop/utils/buffer_factory_shmem.h"

#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <utility>

#include "absl/base/nullability.h"
#include "absl/log/check.h"
#include "absl/numeric/int128.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "filament/libs/utils/include/utils/ashmem.h"
#include "core/split_engine/desktop/utils/buffer_factory.h"

namespace imp::split_engine {
namespace {
// ShmemBuffer represents a buffer backed by a shared memory region.
// Shared memory region is created in ShmemBufferFactory::CreateBuffer.
// ShmemBuffer owns the shared memory region and is responsible for freeing it.
class ShmemBuffer : public Buffer {
 public:
  using ReleaseFunction = std::function<void()>;

  ShmemBuffer(int fd, size_t size_in_bytes, void* /*absl_nonnull*/ mmapped_ptr,
              ReleaseFunction&& release_function)
      : fd_(fd),
        size_in_bytes_(size_in_bytes),
        release_function_(std::move(release_function)),
        mmapped_ptr_(mmapped_ptr) {
    
    
  }
  ShmemBuffer(ShmemBuffer&& other) = delete;
  ShmemBuffer& operator=(ShmemBuffer&& other) = delete;
  ShmemBuffer(const ShmemBuffer&) = delete;
  ShmemBuffer& operator=(const ShmemBuffer&) = delete;
  ~ShmemBuffer() override {
    

    ::munmap(mmapped_ptr_, size_in_bytes_);
    close(fd_);

    release_function_();
  }

  const uint8_t* /*absl_nonnull*/ Data() const noexcept override {
    return reinterpret_cast<const uint8_t*>(mmapped_ptr_);
  }
  uint8_t* /*absl_nonnull*/ Data() noexcept override {
    return reinterpret_cast<uint8_t*>(mmapped_ptr_);
  }
  size_t Size() const noexcept override { return size_in_bytes_; }

 private:
  const int fd_ = -1;
  const size_t size_in_bytes_;
  const ReleaseFunction release_function_;
  void* mmapped_ptr_;
};

}  // namespace

ShmemBufferFactory::ShmemBufferFactory(size_t quota_bytes)
    : quota_bytes_(quota_bytes) {}

absl::StatusOr</*absl_nonnull*/ std::unique_ptr<Buffer>>
ShmemBufferFactory::CreateBuffer(size_t size_in_bytes) noexcept {
  if (size_in_bytes == 0) {
    return absl::InvalidArgumentError("Buffer size must be positive.");
  }

  int fd = -1;
  void* mmapped_ptr = nullptr;

  // Enforce the quota.
  {
    absl::MutexLock lock(&mutex_);
    // Enforce the quota.
    const absl::uint128 used_bytes = used_bytes_;
    const absl::uint128 size_bytes = size_in_bytes;
    if (used_bytes + size_bytes > quota_bytes_) {
      return absl::ResourceExhaustedError("Memory quota exceeded.");
    }

    fd = ::utils::ashmem_create_region("ShmemBuffer", size_in_bytes);
    if (fd == 0) {
      return absl::InternalError("Failed to create shared memory region.");
    }
    // Map the shared memory region to the process address space.
    mmapped_ptr = ::mmap(nullptr, size_in_bytes, PROT_READ | PROT_WRITE,
                         MAP_SHARED, fd, 0);
    if (mmapped_ptr == MAP_FAILED) {
      close(fd);
      return absl::InternalError("Failed to mmap shared memory region.");
    }

    // Update the used bytes.
    used_bytes_ += size_in_bytes;
  }

  // Create a shared memory region.

  return std::make_unique<ShmemBuffer>(fd, size_in_bytes, mmapped_ptr,
                                       [this, size_in_bytes]() {
                                         // Decrement the used bytes when
                                         // ShmemBuffer is destroyed.
                                         absl::MutexLock lock(&mutex_);
                                         
                                         used_bytes_ -= size_in_bytes;
                                       });
}
}  // namespace imp::split_engine
