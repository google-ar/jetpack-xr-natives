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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_SESSION_MEMORY_MANAGER_SHMEM_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_SESSION_MEMORY_MANAGER_SHMEM_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/flatbuffer_arena_allocator.h"
#include "core/split_engine/transport/transport_session_memory_manager.h"

namespace imp::split_engine {

class TransportSessionMemoryManagerImpl : public TransportSessionMemoryManager {
 public:
  class MemoryAllocator {
   public:
    virtual ~MemoryAllocator() = default;

    virtual uint8_t* Allocate(size_t size) = 0;
    virtual void Deallocate(uint8_t* ptr) = 0;
  };

  TransportSessionMemoryManagerImpl(
      std::unique_ptr<imp::ArenaAllocator> arena_allocator,
      imp::BorrowedPtr<MemoryAllocator> memory_allocator)
      : memory_allocator_(std::move(memory_allocator)),
        arena_allocator_(std::move(arena_allocator)) {}

  ~TransportSessionMemoryManagerImpl() override = default;

  TransportSessionMemoryManagerImpl(const TransportSessionMemoryManagerImpl&) =
      delete;
  TransportSessionMemoryManagerImpl& operator=(
      const TransportSessionMemoryManagerImpl&) = delete;
  TransportSessionMemoryManagerImpl(TransportSessionMemoryManagerImpl&&) =
      default;
  TransportSessionMemoryManagerImpl& operator=(
      TransportSessionMemoryManagerImpl&&) = default;

  ArenaHandle CreateArena(size_t size) override {
    const ArenaHandle arena_handle = arena_allocator_->CreateArena(
        size, {AllocateMemory, DeallocateMemory, this});
    // ArenaAllocator returns handles >= 0. Add extra check here to catch
    // implementation change.
    
    return arena_handle;
  }

  uint8_t* GetArenaHead(ArenaHandle arena_handle) override {
    if (arena_handle == kHeapArenaHandle) {
      return nullptr;
    }
    return static_cast<uint8_t*>(arena_allocator_->GetArenaHead(arena_handle));
  }

  size_t GetArenaSize(ArenaHandle arena_handle) override {
    if (arena_handle == kHeapArenaHandle) {
      return std::numeric_limits<size_t>::max();
    }
    return arena_allocator_->GetArenaSize(arena_handle);
  }

  uint8_t* AllocateArenaMemory(ArenaHandle arena_handle, size_t size) override {
    if (arena_handle == kHeapArenaHandle) {
      return new uint8_t[size];
    }
    return arena_allocator_->AllocateArenaMemory(arena_handle, size);
  }

  void DeallocateArenaMemory(ArenaHandle arena_handle, uint8_t* ptr) override {
    if (arena_handle == kHeapArenaHandle) {
      delete[] ptr;
      return;
    }
    /* does nothing, see imp::ArenaAllocator for details */
  }

  void DestroyArena(ArenaHandle arena_handle, bool allow_recycle) override {
    // HeapArena was never created by `CreateArena`, so it shall never be
    // destroyed by `DestroyArena`.
    
    arena_allocator_->DestroyArena(arena_handle, allow_recycle);
  }

 private:
  static uint8_t* AllocateMemory(size_t size, void* user_data) {
    auto* self = static_cast<TransportSessionMemoryManagerImpl*>(user_data);
    return self->memory_allocator_->Allocate(size);
  }

  static void DeallocateMemory(uint8_t* ptr, void* user_data) {
    auto* self = static_cast<TransportSessionMemoryManagerImpl*>(user_data);
    self->memory_allocator_->Deallocate(ptr);
  }

 private:
  const imp::BorrowedPtr<MemoryAllocator> memory_allocator_;
  const std::unique_ptr<imp::ArenaAllocator> arena_allocator_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_TRANSPORT_SESSION_MEMORY_MANAGER_SHMEM_H_
