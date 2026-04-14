// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_BASE_POOL_ALLOCATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_BASE_POOL_ALLOCATOR_H_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <utility>
#include <vector>

#include "absl/container/inlined_vector.h"
#include "absl/hash/hash.h"
#include "absl/log/check.h"
#include "absl/numeric/bits.h"
#include "core/common/pool_allocator_helpers.h"

namespace imp {

// A key used to lookup a pointer stored by the PoolAllocator.
class PoolAllocatorKey {
 public:
  // The value stores both the slot and generation of the key.
  using ValueType = uint32_t;

  // The first 17 bits are used to store the slot index.
  // One slot (empty) is reserved for invalid keys.
  // That means there is a maximum of (2^17 - 1) slots (131,071).
  //
  // This matches the maximum number of entities allowed in Filament
  // (see filament/libs/utils/include/utils/EntityManager.h).
  //
  // The PoolAllocator is used for Impress nodes and components, so this limit
  // is aligned with the maximum number of nodes in a scene. Given that a node
  // can only have one component of each type and each component type gets its
  // own PoolAllocator, no more bits are needed to represent the number of
  // components per node.
  using SlotType = uint32_t;

  // The remaining 15 bits are used to store the generation.
  // That means there is a maximum of 2^15 generations (32,768) per slot.
  using GenerationType = uint16_t;

  // Creates an empty key.
  constexpr PoolAllocatorKey() : value_(0) {}

  constexpr explicit PoolAllocatorKey(ValueType value) : value_(value) {}

  constexpr PoolAllocatorKey(SlotType slot, GenerationType generation)
      : value_((slot & kSlotMask) |
               ((generation << kGenerationShift) & kGenerationMask)) {}

  // Checks if the Key is empty.
  constexpr explicit operator bool() const { return value_ != 0; }

  constexpr bool operator==(PoolAllocatorKey key) const {
    return value_ == key.value_;
  }
  constexpr bool operator!=(PoolAllocatorKey key) const {
    return value_ != key.value_;
  }

  constexpr SlotType GetSlot() const {
    // Slots are stored internally 1 above their actually value because 0 is
    // reserved for empty keys.
    return (value_ & kSlotMask) - 1;
  }
  constexpr GenerationType GetGeneration() const {
    return (value_ & kGenerationMask) >> kGenerationShift;
  }

  template <typename H>
  friend H AbslHashValue(H hash, const PoolAllocatorKey& key) {
    return H::combine(std::move(hash), absl::HashOf(key.value_));
  }

 private:
  static constexpr int kGenerationShift = 17;
  static constexpr ValueType kSlotMask = (1 << kGenerationShift) - 1;
  static constexpr ValueType kGenerationMask = ~kSlotMask;

  ValueType value_;
};

// Base class for PoolAllocator providing the core allocation logic.
//
// This is separated from PoolAllocator to reduce code size from template
// instantiations for each different T the allocator is used with.
//
// Critically, making the logic independent of type T *can* introduce
// performance overhead because it stops the compiler from being able to
// optimize certain operations (i.e. not knowing the size of slots at compile
// time). Which logic is in which class has been chosen very carefully based on
// benchmarking.
template <bool EnableGenerations = true, uint32_t MaxBlockPower = 4>
class BasePoolAllocator {
 public:
  static constexpr uint32_t kMaxBlockPower = MaxBlockPower;
  static constexpr uint32_t kSmallBlocksCapacity =
      (uint32_t{1} << kMaxBlockPower) - 1;

  ~BasePoolAllocator();

  // Checks if a key is valid. Returns false if the key is invalid or stale
  // (generation mismatch).
  bool IsKeyValid(PoolAllocatorKey key) const;

  // Returns the number of allocated objects.
  uint32_t GetAllocatedCount() const;

  // Returns the total capacity of all blocks.
  uint32_t GetCapacity() const;

 protected:
  explicit BasePoolAllocator(
      imp_pool_allocator_internal::MemoryLayout memory_layout);

  // Claims a slot from the pool and returns a pointer to it.
  //
  // The index of the slot is written to out_index.
  //
  // Does *not* update the occupancy, that is done by PoolAllocator for
  // performance.
  void* ClaimSlot(uint32_t* out_index);

  // Checks if the object is in the small blocks region and returns true if it
  // is.
  //
  // If true, the index and pointer to the start of the small block are written
  // to out_small_block_start_index and out_small_block_start_ptr.
  //
  // Does *not* update the small_block_pointers_ array, that is done by
  // PoolAllocator for performance.
  bool FindInSmallBlock(std::byte* obj_bytes,
                        uint32_t* out_small_block_start_index,
                        std::byte** out_small_block_start_ptr) const;

  // Releases a slot back to the pool.
  //
  // Does *not* update the occupancy, that is done by PoolAllocator for
  // performance.
  void ReleaseSlot(void* obj, uint32_t index);

  // Returns a pointer to the slot with the given key.
  //
  // Returns nullptr if the key is invalid or stale (generation mismatch).
  void* GetSlotPointer(PoolAllocatorKey key) const;

  void AddSmallBlock();

  void AddPage();

  const imp_pool_allocator_internal::MemoryLayout memory_layout_;

  uint32_t allocated_count_;
  uint32_t current_capacity_;

  // Free List (Reuse)
  imp_pool_allocator_internal::FreeSlot* free_head_ptr_;
  imp_pool_allocator_internal::FreeSlot* free_tail_ptr_;

  // Stores the generation for each slot. This is used to detect stale keys.
  std::vector<uint16_t> generations_;

  // Bump Cursor (New)
  std::byte* cursor_ptr_;
  uint32_t cursor_index_;

  // Used to detect if we are iterating over the pool so that we can handle
  // allocations/deallocations during iteration correctly.
  uint32_t iteration_depth_;

  // Incremented each time Deallocate is called.
  //
  // This is used in ForEach as a performance optimization to avoid
  // checking the occupancy bit vector for each element, which is more
  // predictable for the CPU.
  uint32_t deallocation_generation_;

  absl::InlinedVector<imp_pool_allocator_internal::SmallBlock, kMaxBlockPower>
      small_blocks_;
  std::vector<std::byte*> pages_;

  // Stores a sparse array of pointers for each slot in the small blocks.
  //
  // This is used for iteration and occupancy in the small blocks region, which
  // is faster than using a bitmask to iterate over the small blocks because
  // they are so small we end up spending most of our time doing boundary
  // checking.
  std::array<void*, kSmallBlocksCapacity> small_block_pointers_;
};

template <bool EnableGenerations, uint32_t MaxBlockPower>
BasePoolAllocator<EnableGenerations, MaxBlockPower>::BasePoolAllocator(
    imp_pool_allocator_internal::MemoryLayout memory_layout)
    : memory_layout_(memory_layout),
      allocated_count_(0),
      current_capacity_(0),
      free_head_ptr_(nullptr),
      free_tail_ptr_(nullptr),
      generations_(kSmallBlocksCapacity),
      cursor_ptr_(nullptr),
      cursor_index_(0),
      iteration_depth_(0),
      deallocation_generation_(0),
      small_block_pointers_() {
  static_assert(MaxBlockPower > 0, "MaxBlockPower must be at least 1");

  // Create the first block immediately, assume the allocator will be used.
  AddSmallBlock();
}

template <bool EnableGenerations, uint32_t MaxBlockPower>
BasePoolAllocator<EnableGenerations, MaxBlockPower>::~BasePoolAllocator() {
  

  for (std::byte* page : pages_) {
    // std::free is used because the memory was allocated with posix_memalign.
    std::free(page);
  }

  for (imp_pool_allocator_internal::SmallBlock& block : small_blocks_) {
    // If T is an over-aligned type (i.e. its alignment requirement is
    // stricter than new guarantees via __STDCPP_DEFAULT_NEW_ALIGNMENT__
    // typically 8 or 16), then `AddSmallBlock` used `posix_memalign`.
    // We must use `free()` to clean it up since it wasn't allocated via `new`.
    if (memory_layout_.slot_alignment > __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
      std::free(block.start);
    } else {
      delete[] block.start;
    }
  }
}

template <bool EnableGenerations, uint32_t MaxBlockPower>
void* BasePoolAllocator<EnableGenerations, MaxBlockPower>::ClaimSlot(
    uint32_t* out_index) {
  uint32_t index;
  void* ptr;

  if (cursor_index_ < current_capacity_) {
    // 1. Prioritize Bump Cursor as long as it doesn't require expanding
    // capacity. This slows down generation reuse (improving safety) and
    // provides better data locality for sequential allocations.
    index = cursor_index_++;
    ptr = cursor_ptr_;
    cursor_ptr_ =
        reinterpret_cast<std::byte*>(ptr) + memory_layout_.slot_size_bytes;
  } else if (free_head_ptr_ != nullptr && iteration_depth_ == 0) {
    // 2. Use Free List (Reuse)
    // Only reuse slots if we have exhausted the current capacity and we are not
    // iterating. This avoids reentrancy issues.
    ptr = free_head_ptr_;
    index = free_head_ptr_->index;

    // Update free head to next
    free_head_ptr_ = free_head_ptr_->next_free_ptr;
    if (free_head_ptr_ == nullptr) {
      free_tail_ptr_ = nullptr;
    }
  } else {
    // 3. Expand Capacity
    if (current_capacity_ < kSmallBlocksCapacity) {
      AddSmallBlock();
    } else {
      AddPage();
    }
    // AddSmallBlock/AddPage updates capacity and sets cursor_ptr_ to the start
    // of the new block/page.
    index = cursor_index_++;
    ptr = cursor_ptr_;
    cursor_ptr_ += memory_layout_.slot_size_bytes;
  }

  allocated_count_++;
  *out_index = index;
  return ptr;
}

template <bool EnableGenerations, uint32_t MaxBlockPower>
bool BasePoolAllocator<EnableGenerations, MaxBlockPower>::FindInSmallBlock(
    std::byte* obj_bytes, uint32_t* out_small_block_start_index,
    std::byte** out_small_block_start_ptr) const {
  uint32_t small_block_start_index = 0;
  for (size_t i = 0; i < small_blocks_.size(); ++i) {
    const imp_pool_allocator_internal::SmallBlock& block = small_blocks_[i];
    if (obj_bytes >= block.start && obj_bytes < block.end) {
      *out_small_block_start_index = small_block_start_index;
      *out_small_block_start_ptr = block.start;
      return true;
    }
    small_block_start_index += block.count;
  }
  return false;
}

template <bool EnableGenerations, uint32_t MaxBlockPower>
void BasePoolAllocator<EnableGenerations, MaxBlockPower>::ReleaseSlot(
    void* obj, uint32_t index) {
  deallocation_generation_++;
  allocated_count_--;

  if constexpr (EnableGenerations) {
    // Increment generation and keep it within 15 bits (0..32767).
    //
    // Note that if the generation wraps around, a stale key might incorrectly
    // resolve to a new object. This requires the slot to be reused 32,768
    // times, which is considered sufficiently unlikely. As a comparison,
    // filament entities work the same way and use 8 bits for the generation,
    // causing rollover after 256 uses.
    static constexpr uint16_t kMaxGenerationMask = (1 << 15) - 1;
    generations_[index] = (generations_[index] + 1) & kMaxGenerationMask;
  }

  // Push to free list (FIFO: push to tail)
  //
  // Placement new is used to construct the FreeSlot in place of the object to
  // store intrusive free list information. Since this is preallocated memory &
  // a trivial type, this is essentially free.
  imp_pool_allocator_internal::FreeSlot* slot =
      new (obj) imp_pool_allocator_internal::FreeSlot();
  slot->next_free_ptr = nullptr;
  slot->index = index;

  if (free_tail_ptr_) {
    free_tail_ptr_->next_free_ptr = slot;
    free_tail_ptr_ = slot;
  } else {
    // List was empty
    free_head_ptr_ = slot;
    free_tail_ptr_ = slot;
  }
}

template <bool EnableGenerations, uint32_t MaxBlockPower>
void* BasePoolAllocator<EnableGenerations, MaxBlockPower>::GetSlotPointer(
    PoolAllocatorKey key) const {
  static_assert(EnableGenerations);

  uint32_t slot = key.GetSlot();

  // This simultaneously tests that the slot is non-empty and  within range
  // using a single check. For an empty key, GetSlot() returns UINT32_MAX,
  // which will fail the capacity check. The max number of slots is lower than
  // UINT32_MAX, so this is always safe.
  if (slot >= GetCapacity()) {
    return nullptr;
  }

  // Ensure key isn't stale by validating that the generation matches.
  if (generations_[slot] != key.GetGeneration()) {
    return nullptr;
  }

  // Find the slot in the small blocks or pages.
  if (slot < kSmallBlocksCapacity) {
    uint32_t block_idx = absl::bit_width(slot + 1) - 1;
    const imp_pool_allocator_internal::SmallBlock& block =
        small_blocks_[block_idx];
    uint32_t start_index = (uint32_t{1} << block_idx) - 1;
    uint32_t offset_idx = slot - start_index;
    return block.start + offset_idx * memory_layout_.slot_size_bytes;
  } else {
    uint32_t relative_idx = slot - kSmallBlocksCapacity;
    uint32_t page_idx = relative_idx / memory_layout_.slots_per_page;
    uint32_t slot_idx = relative_idx % memory_layout_.slots_per_page;
    std::byte* page_mem = pages_[page_idx];
    std::byte* slots_start = page_mem + memory_layout_.slots_start_offset_bytes;
    return slots_start + slot_idx * memory_layout_.slot_size_bytes;
  }
}

template <bool EnableGenerations, uint32_t MaxBlockPower>
bool BasePoolAllocator<EnableGenerations, MaxBlockPower>::IsKeyValid(
    PoolAllocatorKey key) const {
  static_assert(EnableGenerations);

  uint32_t slot = key.GetSlot();

  // This simultaneously tests that the slot is non-empty and  within range
  // using a single check. For an empty key, GetSlot() returns UINT32_MAX, which
  // will fail the capacity check. The max number of slots is lower than
  // UINT32_MAX, so this is always safe.
  if (slot >= GetCapacity()) {
    return false;
  }

  return key.GetGeneration() == generations_[slot];
}

template <bool EnableGenerations, uint32_t MaxBlockPower>
uint32_t
BasePoolAllocator<EnableGenerations, MaxBlockPower>::GetAllocatedCount() const {
  return allocated_count_;
}

template <bool EnableGenerations, uint32_t MaxBlockPower>
uint32_t BasePoolAllocator<EnableGenerations, MaxBlockPower>::GetCapacity()
    const {
  return current_capacity_;
}

template <bool EnableGenerations, uint32_t MaxBlockPower>
void BasePoolAllocator<EnableGenerations, MaxBlockPower>::AddSmallBlock() {
  uint32_t block_idx = small_blocks_.size();
  uint32_t count = uint32_t{1} << block_idx;
  uint32_t bytes = count * memory_layout_.slot_size_bytes;

  std::byte* mem;

  // If T is an over-aligned type, we must use aligned allocation to ensure the
  // memory block adheres to T's alignment requirements. Otherwise,
  // standard new[] is sufficient.
  if (memory_layout_.slot_alignment > __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
    void* mem_ptr = nullptr;
    // Ensure the size is a multiple of the alignment.
    size_t aligned_bytes = (bytes + memory_layout_.slot_alignment - 1) &
                           ~(memory_layout_.slot_alignment - 1);
    if (posix_memalign(&mem_ptr, memory_layout_.slot_alignment,
                       aligned_bytes) != 0) {
      mem_ptr = nullptr;
    }
    mem = static_cast<std::byte*>(mem_ptr);
  } else {
    mem = new (std::nothrow) std::byte[bytes];
  }

  cursor_ptr_ = mem;
  small_blocks_.push_back({mem, mem + bytes, count});
  current_capacity_ += count;

  // Note: We do not need to resize generations_ here because we make space
  // for kSmallBlocksCapacity elements in the constructor.
}

template <bool EnableGenerations, uint32_t MaxBlockPower>
void BasePoolAllocator<EnableGenerations, MaxBlockPower>::AddPage() {
  // Page-size alignment is required because PoolAllocator::{Deallocate,
  // Allocate} use bitmasking based on page_size_bytes to find the page base
  // address from a slot pointer.
  void* page_ptr = nullptr;
  if (posix_memalign(&page_ptr, memory_layout_.page_size_bytes,
                     memory_layout_.page_size_bytes) != 0) {
    page_ptr = nullptr;
  }
  std::byte* page_base = static_cast<std::byte*>(page_ptr);

  uint32_t start_idx = GetCapacity();

  new (page_base) imp_pool_allocator_internal::PageHeader{start_idx};

  uint64_t* occupancy =
      imp_pool_allocator_internal::GetPageOccupancy(page_base);
  new (occupancy) uint64_t[memory_layout_.bitmap_words_per_page]();

  pages_.push_back(page_base);
  current_capacity_ += memory_layout_.slots_per_page;

  cursor_ptr_ = page_base + memory_layout_.slots_start_offset_bytes;

  if constexpr (EnableGenerations) {
    generations_.resize(current_capacity_, 0);
  }
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_BASE_POOL_ALLOCATOR_H_
