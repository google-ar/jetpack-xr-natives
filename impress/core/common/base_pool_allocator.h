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
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <new>
#include <utility>
#include <vector>

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
      // Store slot + 1 because keys interpret 0 as empty.
      : value_(((slot + 1) & kSlotMask) |
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
template <bool EnableGenerations = true, uint32_t MaxBlockPower = 4,
          size_t NumSubsets = 0>
class BasePoolAllocator {
 public:
  static constexpr uint32_t kMaxBlockPower = MaxBlockPower;
  static constexpr uint32_t kSmallBlocksCapacity =
      (uint32_t{1} << kMaxBlockPower) - 1;
  static constexpr size_t kOccupiedSubset = std::numeric_limits<size_t>::max();

  ~BasePoolAllocator();

  // Checks if a key is valid. Returns false if the key is invalid or stale
  // (generation mismatch).
  bool IsKeyValid(PoolAllocatorKey key) const;

  // Returns the number of allocated objects.
  uint32_t GetAllocatedCount() const;

  // Returns the total capacity of all blocks.
  uint32_t GetCapacity() const;

  // Sets or clears the bit for the specified object in a subset bitmap.
  //
  // Subsets are used to quickly iterate over a sub-category of allocated
  // objects (e.g., active vs ready).
  //
  // subset_idx: The index of the subset bitmap (0 <= subset_idx < NumSubsets).
  // ptr: Pointer to the allocated object.
  // value: True to add the object to the subset, false to remove it.
  //
  // This method is templated in BasePoolAllocator to allow calling with base
  // pointer types (e.g. Component*) instead of the actual type. This allows
  // reduction of template bloat and virtual function calls.
  template <typename T>
  void SetSubsetBit(size_t subset_idx, T* ptr, bool value);

  // Reclaims memory from empty tail pages.
  //
  // Scans pages from the tail and frees those that are completely empty
  // (no active objects). Stops at the first non-empty page.
  //
  // This operation requires filtering the intrusive free list to remove
  // slots belonging to deleted pages, which is an O(N) operation where N is
  // the total number of free slots.
  void Trim();

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

  // Returns a pointer to the page bitmaps for the given page base pointer.
  //
  // The page bitmaps are stored in the footer of the page starting from the
  // first word in the occupancy bitmap.
  //
  // This uses a runtime-stored memory layout (`memory_layout_`) to avoid
  // template binary bloat in cold paths. Prefer using
  // `GetPageBitmapsWithStaticLayout` in derived class hot paths where the
  // layout is compile-time known.
  uint64_t* GetPageBitmapsWithDynamicLayout(std::byte* page_base);

  // Returns a reference to the bitmap word for the given page, subset, and
  // word index.
  uint64_t& GetPageBitmapWord(size_t subset_idx, size_t page_idx,
                              size_t word_idx);
  // =========================================================================
  // Fields are ordered to optimize cache locality for ForEach and avoid padding
  // caused by alignment requirements.
  // =========================================================================

  // Stores bitmasks for small blocks occupancy and subsets.
  //
  // Index 0: Occupancy bitmask (indicates if a slot is allocated).
  // Index 1 to NumSubsets: Bitmasks for each subset.
  uint64_t small_block_subsets_[1 + NumSubsets];

  // Bump cursor for the next allocation.
  uint32_t cursor_index_;

  // Incremented each time Deallocate or SetSubsetBit is called.
  //
  // This is used in ForEach as a performance optimization to avoid
  // checking validation/occupancy bits for each element, which is
  // more predictable for the CPU.
  uint32_t modification_generation_;

  // Tracks the total number of objects allocated across small blocks and pages.
  uint32_t allocated_count_;

  // Tracks the number of pages allocated.
  uint16_t pages_count_;

  // Used to detect if we are iterating over the pool so that we can handle
  // allocations/deallocations during iteration correctly.
  uint8_t iteration_depth_;

  // Tracks the number of small blocks allocated.
  uint8_t small_blocks_allocated_count_;

  // Dynamic array of page base pointers for all allocated pages.
  std::byte** pages_;

  // Stores a sparse array of pointers for each slot in the small blocks.
  void* small_block_pointers_[kSmallBlocksCapacity];

  // The current capacity of the pool across allocated small blocks and pages.
  uint32_t current_capacity_;

  // Free List (Reuse)
  imp_pool_allocator_internal::FreeSlot* free_head_ptr_;
  imp_pool_allocator_internal::FreeSlot* free_tail_ptr_;

  // Pointer to the next slot for the bump cursor.
  std::byte* cursor_ptr_;

  // Stores the generation for each slot. This is used to detect stale keys.
  std::vector<uint16_t> generations_;

  // Stores the memory layout of the pool.
  const imp_pool_allocator_internal::MemoryLayout memory_layout_;

  // Small block base pointers.
  std::byte* small_blocks_[MaxBlockPower];
};

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
BasePoolAllocator<EnableGenerations, MaxBlockPower, NumSubsets>::
    BasePoolAllocator(imp_pool_allocator_internal::MemoryLayout memory_layout)
    : small_block_subsets_(),
      cursor_index_(0),
      modification_generation_(0),
      allocated_count_(0),
      pages_count_(0),
      iteration_depth_(0),
      small_blocks_allocated_count_(0),
      pages_(nullptr),
      small_block_pointers_(),
      current_capacity_(0),
      free_head_ptr_(nullptr),
      free_tail_ptr_(nullptr),
      cursor_ptr_(nullptr),
      generations_(),
      memory_layout_(memory_layout),
      small_blocks_() {
  static_assert(MaxBlockPower > 0, "MaxBlockPower must be at least 1");

  // Initialize the generations vector for the small blocks capacity if
  // generations are enabled.
  if constexpr (EnableGenerations) {
    generations_.resize(kSmallBlocksCapacity);
  }

  // Create the first block immediately, assume the allocator will be used.
  AddSmallBlock();
}

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
BasePoolAllocator<EnableGenerations, MaxBlockPower,
                  NumSubsets>::~BasePoolAllocator() {
  

  // Delete all pages.
  for (uint32_t i = 0; i < pages_count_; ++i) {
    std::byte* page_mem = pages_[i];
    if (memory_layout_.slot_alignment > __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
      ::operator delete(
          page_mem,
          imp_pool_allocator_internal::GetPageSize(memory_layout_, NumSubsets),
          std::align_val_t(memory_layout_.slot_alignment));
    } else {
      delete[] page_mem;
    }
  }
  delete[] pages_;

  // Delete all small blocks.
  for (uint32_t i = 0; i < small_blocks_allocated_count_; ++i) {
    std::byte* small_block = small_blocks_[i];
    uint32_t count = uint32_t{1} << i;
    if (memory_layout_.slot_alignment > __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
      uint32_t bytes = count * memory_layout_.slot_size_bytes;
      ::operator delete(small_block,
                        imp_pool_allocator_internal::GetAlignedBytes(
                            bytes, memory_layout_.slot_alignment),
                        std::align_val_t(memory_layout_.slot_alignment));
    } else {
      delete[] small_block;
    }
  }
}

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
void* BasePoolAllocator<EnableGenerations, MaxBlockPower,
                        NumSubsets>::ClaimSlot(uint32_t* out_index) {
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

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
void BasePoolAllocator<EnableGenerations, MaxBlockPower,
                       NumSubsets>::ReleaseSlot(void* obj, uint32_t index) {
  modification_generation_++;
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

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
void* BasePoolAllocator<EnableGenerations, MaxBlockPower,
                        NumSubsets>::GetSlotPointer(PoolAllocatorKey key)
    const {
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
    std::byte* small_block = small_blocks_[block_idx];
    uint32_t start_index = (uint32_t{1} << block_idx) - 1;
    uint32_t offset_idx = slot - start_index;
    return small_block + offset_idx * memory_layout_.slot_size_bytes;
  } else {
    uint32_t relative_idx = slot - kSmallBlocksCapacity;
    uint32_t page_idx = relative_idx >> memory_layout_.page_slots_power;
    uint32_t slot_idx = relative_idx & memory_layout_.page_slot_mask;
    std::byte* page_mem = pages_[page_idx];
    return page_mem + slot_idx * memory_layout_.slot_size_bytes;
  }
}

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
bool BasePoolAllocator<EnableGenerations, MaxBlockPower,
                       NumSubsets>::IsKeyValid(PoolAllocatorKey key) const {
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

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
uint32_t BasePoolAllocator<EnableGenerations, MaxBlockPower,
                           NumSubsets>::GetAllocatedCount() const {
  return allocated_count_;
}

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
uint32_t BasePoolAllocator<EnableGenerations, MaxBlockPower,
                           NumSubsets>::GetCapacity() const {
  return current_capacity_;
}

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
void BasePoolAllocator<EnableGenerations, MaxBlockPower,
                       NumSubsets>::AddSmallBlock() {
  uint32_t block_idx = small_blocks_allocated_count_;
  
  uint32_t count = uint32_t{1} << block_idx;
  uint32_t bytes = count * memory_layout_.slot_size_bytes;

  std::byte* mem;

  // If T is an over-aligned type, we must use aligned allocation to ensure the
  // memory block adheres to T's alignment requirements. Otherwise,
  // standard new[] is sufficient.
  if (memory_layout_.slot_alignment > __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
    mem = static_cast<std::byte*>(
        ::operator new(imp_pool_allocator_internal::GetAlignedBytes(
                           bytes, memory_layout_.slot_alignment),
                       std::align_val_t(memory_layout_.slot_alignment)));
  } else {
    mem = new std::byte[bytes];
  }

  cursor_ptr_ = mem;
  small_blocks_[block_idx] = mem;
  small_blocks_allocated_count_++;
  current_capacity_ += count;

  // Note: We do not need to resize generations_ here because we make space
  // for kSmallBlocksCapacity elements in the constructor.
}

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
void BasePoolAllocator<EnableGenerations, MaxBlockPower,
                       NumSubsets>::AddPage() {
  size_t footer_size = imp_pool_allocator_internal::GetFooterSize(
      memory_layout_.page_slots_power, NumSubsets);
  uint32_t page_bytes =
      memory_layout_.slots_per_page * memory_layout_.slot_size_bytes +
      footer_size;
  std::byte* page_mem;

  if (memory_layout_.slot_alignment > __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
    page_mem = static_cast<std::byte*>(
        ::operator new(imp_pool_allocator_internal::GetAlignedBytes(
                           page_bytes, memory_layout_.slot_alignment),
                       std::align_val_t(memory_layout_.slot_alignment)));
  } else {
    page_mem = new std::byte[page_bytes];
  }

  // Zero-initialize and start the lifetime of the footer bitmaps
  // without array cookie overhead.
  uint64_t* bitmaps = reinterpret_cast<uint64_t*>(
      page_mem + memory_layout_.footer_offset_bytes);
  size_t bitmap_words = footer_size / sizeof(uint64_t);
  for (size_t i = 0; i < bitmap_words; ++i) {
    new (bitmaps + i) uint64_t{0};
  }

  size_t new_pages_count = pages_count_ + 1;
  size_t old_pages_count = pages_count_;

  // Grow page base pointers array
  std::byte** new_memories = new std::byte*[new_pages_count];
  if (pages_) {
    std::copy(pages_, pages_ + old_pages_count, new_memories);
    delete[] pages_;
  }
  new_memories[old_pages_count] = page_mem;
  pages_ = new_memories;
  pages_count_ = new_pages_count;

  current_capacity_ += memory_layout_.slots_per_page;
  cursor_ptr_ = page_mem;

  if constexpr (EnableGenerations) {
    // current_capcity_ can be smaller than generations_.size() if we have
    // trimmed the pages_.
    if (generations_.size() < current_capacity_) {
      generations_.resize(current_capacity_, 0);
    }
  }
}

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
uint64_t* BasePoolAllocator<EnableGenerations, MaxBlockPower, NumSubsets>::
    GetPageBitmapsWithDynamicLayout(std::byte* page_base) {
  return reinterpret_cast<uint64_t*>(page_base +
                                     memory_layout_.footer_offset_bytes);
}

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
uint64_t& BasePoolAllocator<EnableGenerations, MaxBlockPower,
                            NumSubsets>::GetPageBitmapWord(size_t subset_idx,
                                                           size_t page_idx,
                                                           size_t word_idx) {
  uint64_t* bitmaps = GetPageBitmapsWithDynamicLayout(pages_[page_idx]);
  return bitmaps[subset_idx * memory_layout_.page_words_per_subset + word_idx];
}

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
void BasePoolAllocator<EnableGenerations, MaxBlockPower, NumSubsets>::Trim() {
  // Safely skip if we are iterating to avoid invalidating iterator state.
  if (iteration_depth_ > 0) {
    return;
  }

  // Iterate backwards through pages to reclaim empty ones.
  uint32_t pages_to_trim = 0;
  uint32_t first_trimmed_index = current_capacity_;

  // 1. Identify contiguous empty tail pages.
  for (size_t i = pages_count_; i > 0; --i) {
    size_t idx = i - 1;

    bool is_empty = true;
    // Subset 0 (occupancy bitmap) is first bitmap.
    uint64_t* bitmaps = GetPageBitmapsWithDynamicLayout(pages_[idx]);
    for (uint32_t w = 0; w < memory_layout_.page_words_per_subset; ++w) {
      if (bitmaps[w] != 0) {
        is_empty = false;
        break;
      }
    }

    if (!is_empty) {
      break;
    }

    first_trimmed_index =
        kSmallBlocksCapacity + idx * memory_layout_.slots_per_page;
    pages_to_trim++;
  }

  // 2. Filter the free list in a single pass if we have pages to trim.
  if (pages_to_trim > 0) {
    imp_pool_allocator_internal::FreeSlot* new_head = nullptr;
    imp_pool_allocator_internal::FreeSlot* new_tail = nullptr;

    imp_pool_allocator_internal::FreeSlot* curr = free_head_ptr_;
    while (curr != nullptr) {
      if (curr->index < first_trimmed_index) {
        if (new_head == nullptr) {
          new_head = curr;
        } else {
          new_tail->next_free_ptr = curr;
        }
        new_tail = curr;
      }
      curr = curr->next_free_ptr;
    }

    if (new_tail != nullptr) {
      new_tail->next_free_ptr = nullptr;
    }

    free_head_ptr_ = new_head;
    free_tail_ptr_ = new_tail;

    // 3. Delete pages and update capacity.
    for (uint32_t i = 0; i < pages_to_trim; ++i) {
      std::byte* page_base = pages_[pages_count_ - 1 - i];
      if (memory_layout_.slot_alignment > __STDCPP_DEFAULT_NEW_ALIGNMENT__) {
        ::operator delete(page_base,
                          imp_pool_allocator_internal::GetPageSize(
                              memory_layout_, NumSubsets),
                          std::align_val_t(memory_layout_.slot_alignment));
      } else {
        delete[] page_base;
      }
    }

    size_t new_pages_count = pages_count_ - pages_to_trim;

    // Shrink page base pointers array
    if (new_pages_count > 0) {
      std::byte** new_memories = new std::byte*[new_pages_count];
      std::copy(pages_, pages_ + new_pages_count, new_memories);
      delete[] pages_;
      pages_ = new_memories;
    } else {
      delete[] pages_;
      pages_ = nullptr;
    }
    pages_count_ = new_pages_count;

    current_capacity_ = first_trimmed_index;

    // Roll back the bump cursor if it was pointing into the deleted pages or
    // ran past them.
    if (cursor_index_ > current_capacity_) {
      cursor_index_ = current_capacity_;
      // Note: We don't need to update cursor_ptr_ here.
      // - If cursor_index_ was in a deleted page, setting it to
      //   current_capacity_ ensures that the next allocation will NOT use the
      //   bump cursor (since cursor_index_ == current_capacity_) and will
      //   instead call AddPage(), which resets cursor_ptr_ to the newly
      //   allocated page.
      // - If cursor_index_ was in an earlier page, it was less than
      //   current_capacity_ and we didn't touch it, so cursor_ptr_ is still
      //   naturally correct.
    }
  }

  // NOTE: The generations_ vector is intentionally not trimmed.
  // If the allocator grows again and re-uses those slot indices, the generation
  // tracking must remain consistent.
}

template <bool EnableGenerations, uint32_t MaxBlockPower, size_t NumSubsets>
template <typename T>
void BasePoolAllocator<EnableGenerations, MaxBlockPower,
                       NumSubsets>::SetSubsetBit(size_t subset_idx, T* ptr,
                                                 bool value) {
  

  PoolAllocatorKey key = ptr->GetPoolAllocatorKey();
  uint32_t slot = key.GetSlot();

  if (slot < kSmallBlocksCapacity) {
    uint64_t bit = (1ULL << slot);
    // Intentionally skip checking if the value has changed, instead rely on
    // the caller not calling this unnecessarily. This is because it is used in
    // performance critical code so we avoid excessive branching.
    uint64_t old_val = small_block_subsets_[1 + subset_idx];
    small_block_subsets_[1 + subset_idx] =
        (old_val & ~bit) | (static_cast<uint64_t>(value) << slot);
    modification_generation_++;
  } else {
    uint32_t relative_idx = slot - kSmallBlocksCapacity;
    uint32_t page_idx = relative_idx >> memory_layout_.page_slots_power;
    uint32_t slot_idx = relative_idx & memory_layout_.page_slot_mask;

    size_t word_idx = slot_idx / 64;
    size_t shift = slot_idx % 64;
    uint64_t& word = GetPageBitmapWord(1 + subset_idx, page_idx, word_idx);
    word = (word & ~(1ULL << shift)) | (static_cast<uint64_t>(value) << shift);
    modification_generation_++;
  }
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_BASE_POOL_ALLOCATOR_H_
