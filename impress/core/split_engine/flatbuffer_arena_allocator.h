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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_FLATBUFFER_ARENA_ALLOCATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_FLATBUFFER_ARENA_ALLOCATOR_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "zetasql/base/arena.h"
#include "flatbuffers/allocator.h"

namespace imp {

// An arena-based allocator for use with the flatbuffers used by Split Engine
// IPC. This is intended to simulate the shared memory system that a "real"
// OS-boundary split engine implementation would use.
// The expected usage pattern is that a separate arena is used for every frame
// and deallocated either at the end of the frame or at some later sync point
// when all the data for that frame is finished being consumed.
class FlatbufferArenaAllocator : public flatbuffers::Allocator {
 public:
  using ArenaHandle = int32_t;

  // See the CreateArena function documentation for explanation of these types.
  using LowLevelAllocFunc = void* (*)(size_t size, void* user);
  using LowLevelDeallocFunc = void (*)(void* ptr, void* user);

  // Allocates a new memory arena. All subsequent calls to allocate() will
  // allocate within this arena.
  // By default, the memory blocks backing the arena are ultimately allocated
  // via global operator new and deallocated in DestroyArena via global operator
  // delete. If first_block_alloc and first_block_dealloc are provided, the
  // first block will be allocated via that function instead, and deallocated by
  // the latter in DestroyArena (if allow_recycle=false), or when this
  // FlatbufferArenaAllocator object is destructed, whichever comes first.
  // Three important points to note:
  // 1. This applies to the first block only. If the arena grows to more than
  //    one block, i.e. if the amount of memory allocated is larger than
  //    block_size, then subsequent blocks will still use operator new.
  // 2. If first_block_alloc is provided, then that block (and only that block)
  //    is fully owned by the caller and is never deallocated by this class.
  //    See `allow_recycle` in the DestroyArena function documentation.
  // 3. The first_block_alloc function might not necessarily be called, if an
  //    appropriate recycled arena is found instead.
  ArenaHandle CreateArena(size_t block_size,
                          LowLevelAllocFunc first_block_alloc = nullptr,
                          LowLevelDeallocFunc first_block_dealloc = nullptr,
                          void* user = nullptr);

  // Deallocates a memory arena. It is the caller's responsibility to ensure
  // that no memory allocated from within this arena is still alive after this
  // function is called.
  // If this function is called on the currently active arena, i.e. the handle
  // returned from the most recent call to CreateArena, then CreateArena MUST be
  // called again before any subsequent calls to allocate or deallocate. However
  // it is advised to avoid that state and never call DestroyArena on the
  // currently active arena.
  //
  // allow_recycle: Arenas are recycled by default. That is, a subsequent call
  // to CreateArena may return an arena handle that was previously passed to
  // DestroyArena. When an arena is recycled, all but the first block are
  // deallocated. In other words, the first block is NOT deallocated, and in
  // particular this also means that `first_block_dealloc` will not be called.
  void DestroyArena(ArenaHandle arena_handle, bool allow_recycle = true);

  // Returns the number of bytes allocated in the arena. The value is always at
  // least block_size, and is exactly block_size iff the arena contains a single
  // block.
  size_t GetArenaSize(ArenaHandle arena_handle);

  // Returns the address of the beginning of the first block in the arena.
  void* GetArenaHead(ArenaHandle arena_handle);

  // Returns the handle of the currently active arena. If no arena is currently
  // active, returns -1.
  ArenaHandle GetActiveArena() { return active_arena_; }

  // Closes the currently active arena. This prevents any further allocations
  // from being made in this arena, but does not deallocate or recycle any
  // memory until DestroyArena is called.
  //
  // As DestroyArena cannot be called on the currently active arena, this can be
  // used to make an arena eligible for DestroyArena() without allocating a new
  // arena.
  void CloseActiveArena() { active_arena_ = -1; }

  // Allocates memory from the currently active arena.
  // Preconditions: An arena is currently active, which means 1) CreateArena has
  // been called at least once and 2) if DestroyArena was called on an active
  // arena then CreateArena has been called again before any call to allocate or
  // deallocate.
  uint8_t* allocate(size_t size) override;

  // deallocate does nothing. Memory is freed by DestroyArena().
  void deallocate(uint8_t* p, size_t size) override;

 private:
  class ArenaAndAllocFunc {
   public:
    ArenaAndAllocFunc(size_t block_size, LowLevelAllocFunc first_block_alloc,
                      LowLevelDeallocFunc first_block_dealloc, void* user);
    ~ArenaAndAllocFunc();
    ArenaAndAllocFunc(ArenaAndAllocFunc&& other);
    ArenaAndAllocFunc& operator=(ArenaAndAllocFunc&& other);

    zetasql_base::UnsafeArena* Get() { return arena_.get(); }

    // Checks if this arena is eligible to be reused.
    bool IsMatch(size_t block_size, LowLevelAllocFunc first_block_alloc,
                 LowLevelDeallocFunc first_block_dealloc, void* user);

    // Mark the arena as being used.
    void SetInUse();

    // Deallocates all blocks in the arena except the first one, and flags the
    // arena as available for reuse.
    void Reset();

    // Deletes the arena altogether. This ArenaAndAllocFunc object becomes
    // available for reuse, but not the underlying arena itself.
    void Clear();

    // AreanaHead is non-null of a non-null first_block_alloc was pass to ctor.
    void* GetArenaHead() { return first_block_head_; }

   private:
    std::unique_ptr<zetasql_base::UnsafeArena> arena_;
    void* first_block_head_;
    LowLevelAllocFunc first_block_alloc_;
    LowLevelDeallocFunc first_block_dealloc_;
    void* user_;
    bool in_use_;
  };

  std::vector<ArenaAndAllocFunc> arenas_;
  ArenaHandle active_arena_ = -1;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_FLATBUFFER_ARENA_ALLOCATOR_H_
