/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_SHARED_MEMORY_REGION_ALLOCATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_SHARED_MEMORY_REGION_ALLOCATOR_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/synchronization/mutex.h"
#include "core/async/background_scheduler.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/android/bridge_buffer.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/transport/transport_session_memory_manager_impl.h"

namespace imp::split_engine {

// Uses shared memory regions (via BridgeBuffer) to allocate memory.
class SharedMemoryRegionAllocator
    : public TransportSessionMemoryManagerImpl::MemoryAllocator {
 public:
  SharedMemoryRegionAllocator(
      std::unique_ptr<BufferHandleFactory> buffer_handle_factory,
      imp::BorrowedPtr<BackgroundScheduler> scheduler)
      : buffer_handle_factory_(std::move(buffer_handle_factory)),
        scheduler_(scheduler) {};
  ~SharedMemoryRegionAllocator() = default;

  SharedMemoryRegionAllocator(const SharedMemoryRegionAllocator&) = delete;
  SharedMemoryRegionAllocator& operator=(const SharedMemoryRegionAllocator&) =
      delete;
  SharedMemoryRegionAllocator(SharedMemoryRegionAllocator&&) = delete;
  SharedMemoryRegionAllocator& operator=(SharedMemoryRegionAllocator&&) =
      delete;

  // Allocates a shared memory region of at least size_bytes size.
  uint8_t* Allocate(size_t size_bytes) override {
    auto bridge_buffer = std::make_unique<BridgeBuffer>(
        *buffer_handle_factory_, size_bytes, *scheduler_);
    uint8_t* buffer_head = bridge_buffer->DataAs<uint8_t>();
    absl::MutexLock lock(bridge_buffers_mutex_);
    bridge_buffers_.emplace(buffer_head, std::move(bridge_buffer));
    return buffer_head;
  }

  // Deallocates a shared memory region.
  void Deallocate(uint8_t* ptr) override {
    absl::MutexLock lock(bridge_buffers_mutex_);
    // Note that the map that we're erasing from holds unique_ptrs, so this
    // erase() doesn't just remove it from the map but also destroys the
    // BridgeBuffer object.
    bridge_buffers_.erase(ptr);
  }

  const BufferHandleFactory::BufferHandle& GetBufferHandle(uint8_t* ptr) {
    absl::MutexLock lock(bridge_buffers_mutex_);
    const auto it = bridge_buffers_.find(ptr);
    
    return it->second->GetHandle();
  }

 private:
  std::unique_ptr<BufferHandleFactory> buffer_handle_factory_;
  imp::BorrowedPtr<BackgroundScheduler> scheduler_;

  absl::Mutex bridge_buffers_mutex_;
  absl::flat_hash_map<const uint8_t*, std::unique_ptr<BridgeBuffer>>
      bridge_buffers_ ABSL_GUARDED_BY(bridge_buffers_mutex_);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_SHARED_MEMORY_REGION_ALLOCATOR_H_
