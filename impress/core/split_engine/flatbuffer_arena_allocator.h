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
#include "absl/types/span.h"
#include "flatbuffers/allocator.h"
#include "flatbuffers/base.h"

namespace imp {

// An arena-based allocator for use with the flatbuffers used by Split Engine
// IPC. This is intended to simulate the shared memory system that a "real"
// OS-boundary split engine implementation would use.
// The expected usage pattern is that a separate arena is used for every frame
// and deallocated either at the end of the frame or at some later sync point
// when all the data for that frame is finished being consumed.
//
// FlatbufferBuilder will pass this allocator to underlying structure
// (vector_downward). vector_downward will call `allocate` when needed.
//
// Multiple builders can be active at the same time within one arena.
//
class FlatbufferArenaAllocator : public flatbuffers::Allocator {
 public:
  using ArenaHandle = int32_t;

  // See the CreateArena function documentation for explanation of these types.
  using LowLevelAllocFunc = void* (*)(size_t size, void* user);
  using LowLevelDeallocFunc = void (*)(void* ptr, void* user);

  enum class GrowthStrategy : uint8_t {
    // The first block is allocated either via operator new or via the
    // first_block_alloc function if provided. The first block allocated will be
    // the only block allocated. Attempting to allocate memory beyond the first
    // block will result in a CHECK failure.
    kDontGrowBeyondFirstBlock,

    // The first block is allocated either via operator new or via the
    // first_block_alloc function if provided. If more memory is needed,
    // subsequent blocks are allocated via operator new.
    kUnlimitedGrowth,
  };

  struct MemoryOptions {
    // If set, the first block will be allocated via this function instead of
    // operator new. Allocation of consequent blocks is managed by
    // `growth_strategy`.
    LowLevelAllocFunc first_block_alloc = nullptr;
    // If set, the first block will be deallocated via this function instead of
    // operator delete.
    LowLevelDeallocFunc first_block_dealloc = nullptr;

    // User data to be passed to the alloc and dealloc functions.
    void* user = nullptr;

    // The memory allocation mode for subsequent blocks.
    GrowthStrategy growth_strategy = GrowthStrategy::kDontGrowBeyondFirstBlock;

    bool operator==(const MemoryOptions& other) const = default;
    bool operator!=(const MemoryOptions& other) const = default;
  };

  // Allocates a new memory arena. All subsequent calls to allocate() will
  // allocate within this arena.
  // By default, the memory blocks backing the arena are ultimately allocated
  // via global operator new and deallocated in DestroyArena via global operator
  // delete. If first_block_alloc and first_block_dealloc are provided, the
  // first block will be allocated via that function instead, and deallocated by
  // the latter in DestroyArena (if allow_recycle=false), or when this
  // FlatbufferArenaAllocator object is destructed, whichever comes first.
  // Three important points to note:
  // 1. This applies to the first block only. If
  //    `GrowthStrategy::kUnlimitedGrowth` is used and the arena grows to more
  //    than one block, i.e. if the amount of memory allocated is larger than
  //    block_size, then subsequent blocks will still use operator new.
  //
  //    If `GrowthStrategy::kDontGrowBeyondFirstBlock` is used, then attempt to
  //    allocate memory beyond the `block_size` will result in CHECK failure
  //
  // 2. If first_block_alloc is provided, then that block (and only that block)
  //    is fully owned by the caller and is never deallocated by this class.
  //    See `allow_recycle` in the DestroyArena function documentation.
  //
  // 3. The first_block_alloc function might not necessarily be called, if an
  //    appropriate recycled arena is found instead.
  virtual ArenaHandle CreateArena(size_t block_size,
                                  MemoryOptions memory_options);

  // Creates an arena with default memory options.
  //
  // Workaround for https://github.com/llvm/llvm-project/issues/36032
  //
  // Cannot use '= {}' in previous declaration: it will cause a compiler error
  // produced by Clang bug.
  ArenaHandle CreateArena(size_t block_size);

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
    ArenaAndAllocFunc(size_t block_size, MemoryOptions memory_options);
    ~ArenaAndAllocFunc();
    ArenaAndAllocFunc(ArenaAndAllocFunc&& other);
    ArenaAndAllocFunc& operator=(ArenaAndAllocFunc&& other);

    zetasql_base::UnsafeArena* Get() { return arena_.get(); }

    // Checks if this arena is eligible to be reused.
    bool IsMatch(size_t block_size, const MemoryOptions& memory_options);

    // Mark the arena as being used.
    void SetInUse();

    // Deallocates all blocks in the arena except the first one, and flags the
    // arena as available for reuse.
    void Reset();

    // Deletes the arena altogether. This ArenaAndAllocFunc object becomes
    // available for reuse, but not the underlying arena itself.
    void Clear();

    // AreanaHead is non-null of a non-null first_block_alloc was pass to ctor.
    void* GetArenaHead() const { return first_block_head_; }

    GrowthStrategy GetGrowthStrategy() const {
      return memory_options_.growth_strategy;
    }

   private:
    std::unique_ptr<zetasql_base::UnsafeArena> arena_;
    void* first_block_head_;
    MemoryOptions memory_options_;
    bool in_use_;
  };

  std::vector<ArenaAndAllocFunc> arenas_;
  ArenaHandle active_arena_ = -1;
};

// In order to transmit flatbuffers over RPC effectively, we need to prefix them
// with a size field. This can be achieved by calling
// FlatbufferBuilder::FinishSizePrefixed instead of FlatbufferBuilder::Finish
// _EVERYWHERE_. However, this is error prone as it is easy to forget about
// this. In addition, every implementation of receiving side will have to be
// updated to use flatbuffers::GetSizePrefixedRoot instead of
// flatbuffers::GetRoot.
//
// flatbuffers::FlatbufferBuilder is using vector_downward to store flatbuffer
// data. Key difference between vector and vector_downward is that
// vector_downward grows from the end of the buffer. E.g.:
// 1. Allocator::allocate():
//    [ . ][ . ][ . ][ . ][ . ][ . ]
//      |
//      └- allocate() returns this pointer.
//
// 2. vector_downward::push_small(1)
//    [ . ][ . ][ . ][ . ][ . ][ 1 ]
//                               |
//                               └- FlatbufferBuilder::GetBufferPointer()
//
// So, we can utilize the unused space immediately before the flatbuffer to
// store the size of the flatbuffer. In order to handle the case when the whole
// buffer is used, `SizePrefixedFlatbufferArenaAllocator` from below reserves
// few bytes in the beginning of the buffer to handle
// the case when the whole buffer is used by the flatbuffer builder.
//
// 1. SizePrefixedFlatbufferArenaAllocator::allocate():
//  [ RESERVED ][ . ][ . ][ . ][ . ][ . ]
//                |
//                └- allocate() returns this pointer.
// 2. vector_downward::push_small(2)
//  [ RESERVED ][ . ][ . ][ . ][ . ][ 2 ]
//                                    |
//                                    └- FlatbufferBuilder::GetBufferPointer()
//
// 3. SizePrefixedFlatbufferArenaAllocator::PrependSize(1):
//  [ RESERVED ][ . ][ . ][ . ][ 1 ][ 2 ]
//                               └-┬--┘
//                                 └-- PrependSize() returns this span.
//
//
class SizePrefixedFlatbufferArenaAllocator : public FlatbufferArenaAllocator {
 public:
  using SizeType = flatbuffers::uoffset_t;
  // Caller is responsible to ensure that `ptr` points to a memory that was
  // allocated by this allocator.
  //
  // Parameters:
  //   ptr: a.k.a. FlatbufferBuilder::GetBufferPointer()
  //   size: a.k.a. FlatbufferBuilder::GetSize()
  //
  // Returns:
  //   A span of size `size + sizeof(SizeType)` containing the size field at the
  //   beginning and the flatbuffer data afterwards.
  //
  absl::Span<const uint8_t> PrependSize(uint8_t* ptr, SizeType size);

  ArenaHandle CreateArena(size_t block_size,
                          MemoryOptions memory_options) override;
  uint8_t* allocate(size_t size) override;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_FLATBUFFER_ARENA_ALLOCATOR_H_
