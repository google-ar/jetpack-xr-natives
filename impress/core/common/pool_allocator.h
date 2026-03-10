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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_POOL_ALLOCATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_POOL_ALLOCATOR_H_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/base/optimization.h"
#include "absl/container/inlined_vector.h"
#include "absl/hash/hash.h"
#include "absl/log/check.h"
#include "absl/numeric/bits.h"

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

// A pool allocator for objects of type T. Provides pointer stability and
// improves data locality compared to individual heap allocations. The API is
// not thread-safe.
//
// This allocator uses a hybrid memory layout to minimize memory waste.
//
// LowCapacity: Geometric Small Blocks
//   - For the first few allocations (indices `0-14` by default, configured via
//   `MaxBlockPower`), memory is allocated in geometrically growing blocks
//   (sizes 1, 2, 4, 8).
//
// HighCapacity: Aligned Paging
//  - Once the "Small Block" capacity is reached, the allocator switches to
//    fixed-size pages (e.g., 4KB, 8KB, up to 64KB depending on item size).
//
// This approach balances memory overhead with allocation frequency:
// - For small numbers of allocations, memory usage is kept low.
// - For large numbers of allocations, allocation frequency is reduced to when a
//   new page is needed.
//
// Memory Retention:
// - The allocator retains memory for the current capacity even after
//   deallocations.
// - Deallocated objects are not immediately freed but are added to a free list
//   for reuse within the same block or page.
// - This avoids memory fragmentation and the overhead of repeated memory
//   allocations/deallocations, at the cost of maintaining a high watermark of
//   peak memory usage.
//
// EnableGenerations controls whether the allocator tracks generations for
// safe handle validation. If true, Allocate calls will return a key with a
// valid generation, and Deallocate calls will increment the generation for
// the slot.
//
// Key Features:
// - Pointer stability
// - O(1) Allocate (Intrusive FreeList).
// - O(1) Deallocate (Page Masking).
// - O(1) ResolveKey (Fast key validation).
// - Minimal memory waste for small element counts.
// - Zero memory overhead for per-item headers.
template <typename T, bool EnableGenerations = true, uint32_t MaxBlockPower = 4>
class PoolAllocator {
 public:
  static constexpr uint32_t kMaxBlockPower = MaxBlockPower;
  static constexpr uint32_t kSmallBlocksCapacity =
      (uint32_t{1} << kMaxBlockPower) - 1;

  struct PtrAndKey {
    // The pointer to the allocated object.
    T* ptr;

    // If EnableGenerations is true, this key can be passed into ResolveKey to
    // lookup ptr later. If ptr has been deallocated, then ResolveKey will
    // return nullptr.
    PoolAllocatorKey key;
  };

  using AllocateResult = std::conditional_t<EnableGenerations, PtrAndKey, T*>;

  PoolAllocator();

  ~PoolAllocator();

  // Allocates and constructs an object of type T.

  template <typename... Args>
  AllocateResult Allocate(Args&&... args);

  // Destructs and deallocates an object of type T.
  void Deallocate(T* obj);

  // Resolves a key to a pointer. Returns nullptr if the key is invalid or
  // stale (generation mismatch).
  T* ResolveKey(PoolAllocatorKey key) const;

  // Checks if a key is valid. Returns false if the key is invalid or stale
  // (generation mismatch).
  bool IsKeyValid(PoolAllocatorKey key) const;

  // Returns the number of allocated objects.
  uint32_t GetAllocatedCount() const;

  // Returns the total capacity of all blocks.
  uint32_t GetCapacity() const;

  // Returns the size of the items stored by this allocator in bytes. Not
  // guaranteed to be the same as sizeof(T) if T is smaller than 12 bytes.
  static constexpr size_t GetItemSize();

  // Iterates over all allocated objects in the pool and calls the given
  // function for each object.
  //
  // It is guaranteed to be safe to recursively call ForEach, and to call
  // Allocate or Deallocate during iteration. Objects allocated during ForEach
  // will not be visited until subsequent ForEach calls.
  //
  // The objects are iterated first through the small blocks in increasing order
  // of their index, and then through the pages in increasing order of their
  // index. This means that order is consistent, but not necessarily in
  // allocation order because indices can be reused after deallocation. This
  // approach ensures that iteration prioritizes cache locality.
  //
  // Example:
  //   allocator.ForEach([](T* obj) { /* do something with obj */ });
  template <typename Fn>
  void ForEach(Fn&& fn);

 private:
  // Stores page layout information determined based on the type T.
  struct PageConfig {
    // The size of each page in bytes.
    uint32_t page_size_bytes;
    // The number of slots in each page.
    uint32_t slots_per_page;
    // The number of 64-bit words needed to store the bitmap storing which slots
    // are occupied.
    uint32_t bitmap_words;
    // The offset from the start of the page to the first slot.
    uint32_t slots_start_offset;
  };

  // Computes the page layout information based on the type T.
  static constexpr PageConfig ComputePageConfig();

  // Forward declaration of the Slot because it is used in the FreeSlot struct
  // before it is fully defined.
  union Slot;

  // FreeSlot stores both it's own index and the pointer to the next free slot.
  // This allows us to manage the free list with zero overhead to calculate the
  // index or the pointer.
  struct FreeSlot {
    Slot* next_free_ptr;
    // This is the overall index of the slot in the pool across all small blocks
    // and pages. This is used by the Allocate function to quickly find the
    // index of the slot from the slot pointer. This saves CPU usage so we don't
    // need to find/calculate it.
    uint32_t index;
  };

  // When Deallocate is called, we use the memory of the slot to intrusively
  // store both the index and pointer of the next free slot in the free list.
  // This allows us to manage the free list with zero lookup overhead.
  union Slot {
    T object;
    FreeSlot free_slot;
  };

  // A contiguous range of slots that is smaller than the page size.
  struct SmallBlock {
    std::byte* start;
    std::byte* end;
    uint32_t count;
  };

  // The size of the slot is the size of the object or the size of the free slot
  // structure, whichever is larger. In practice for Impress's use cases, T is
  // always bigger (imp::Component, imp::NodeController).
  static constexpr size_t kItemSize = sizeof(Slot);

  static constexpr PageConfig kPageConfig = ComputePageConfig();

  // Information stored at the beginning of each page.
  struct PageHeader {
    uint32_t start_index;
    std::array<uint64_t, kPageConfig.bitmap_words> occupancy;
  };

  void AddSmallBlock();

  void AddPage();

  // Given a pointer to an object, return the page header for that object.
  //
  // This assumes that the ptr is part of a page.
  PageHeader* GetPageHeader(T* ptr) const;

  uint32_t allocated_count_;
  uint32_t current_capacity_;

  // Free List (Reuse)
  Slot* free_head_ptr_;
  Slot* free_tail_ptr_;

  // Bump Cursor (New)
  uint32_t cursor_index_;
  T* cursor_ptr_;

  absl::InlinedVector<SmallBlock, kMaxBlockPower> small_blocks_;
  std::vector<PageHeader*> pages_;

  // Stores the generation for each slot. This is used to detect stale keys.
  std::vector<uint16_t> generations_;

  // Stores a sparse array of pointers for each slot in the small blocks.
  //
  // This is used for iteration and occupancy in the small blocks region, which
  // is faster than using a bitmask to iterate over the small blocks because
  // they are so small we end up spending most of our time doing boundary
  // checking.
  std::array<T*, kSmallBlocksCapacity> small_block_pointers_;

  // Used to detect if we are iterating over the pool so that we can handle
  // allocations/deallocations during iteration correctly.
  int iteration_depth_;

  // Incremented each time Deallocate is called.
  //
  // This is used in ForEach as a performance optimization to avoid
  // checking the occupancy bit vector for each element, which is more
  // predictable for the CPU.
  uint32_t deallocation_generation_;
};

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
PoolAllocator<T, EnableGenerations, MaxBlockPower>::PoolAllocator()
    : allocated_count_(0),
      current_capacity_(0),
      free_head_ptr_(nullptr),
      free_tail_ptr_(nullptr),
      cursor_index_(0),
      cursor_ptr_(nullptr),
      generations_(kSmallBlocksCapacity),
      small_block_pointers_(),
      iteration_depth_(0),
      deallocation_generation_(0) {
  // Ensure T's alignment is respected.
  static_assert(sizeof(Slot) + kPageConfig.slots_start_offset <=
                    kPageConfig.page_size_bytes,
                "T is too large for PoolAllocator pages");
  static_assert(MaxBlockPower > 0, "MaxBlockPower must be at least 1");

  // Create the first block immediately, assume the allocator will be used.
  AddSmallBlock();
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
PoolAllocator<T, EnableGenerations, MaxBlockPower>::~PoolAllocator() {
  

  for (void* page : pages_) {
    ::operator delete(page, std::align_val_t{kPageConfig.page_size_bytes});
  }

  for (SmallBlock& block : small_blocks_) {
    // If T is an over-aligned type (i.e. its alignment requirement is
    // stricter than malloc/new guarantees via std::max_align_t), we must
    // use delete with alignment specified, matching the aligned new used
    // to allocate the block in AddSmallBlock().
    if constexpr (alignof(T) > alignof(std::max_align_t)) {
      ::operator delete[](block.start, std::align_val_t(alignof(T)));
    } else {
      delete[] block.start;
    }
  }
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
template <typename... Args>
typename PoolAllocator<T, EnableGenerations, MaxBlockPower>::AllocateResult
PoolAllocator<T, EnableGenerations, MaxBlockPower>::Allocate(Args&&... args) {
  uint32_t index;
  T* ptr;

  if (cursor_index_ < current_capacity_) {
    // 1. Prioritize Bump Cursor as long as it doesn't require expanding
    // capacity. This slows down generation reuse (improving safety) and
    // provides better data locality for sequential allocations.
    index = cursor_index_++;
    ptr = cursor_ptr_;
    cursor_ptr_ =
        reinterpret_cast<T*>(reinterpret_cast<std::byte*>(ptr) + kItemSize);
  } else if (free_head_ptr_ != nullptr && iteration_depth_ == 0) {
    // 2. Use Free List (Reuse)
    // Only reuse slots if we have exhausted the current capacity and we are not
    // iterating. This avoids reentrancy issues.
    ptr = reinterpret_cast<T*>(free_head_ptr_);
    index = free_head_ptr_->free_slot.index;

    // Update free head to next
    free_head_ptr_ = free_head_ptr_->free_slot.next_free_ptr;
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
    cursor_ptr_ =
        reinterpret_cast<T*>(reinterpret_cast<std::byte*>(ptr) + kItemSize);
  }

  allocated_count_++;

  // Construct object using placement new within the slot.
  new (ptr) T(std::forward<Args>(args)...);

  // Track that the slot is occupied.
  if (index < kSmallBlocksCapacity) {
    small_block_pointers_[index] = ptr;
  } else {
    // Update page occupancy
    PageHeader* header = GetPageHeader(ptr);
    size_t offset = reinterpret_cast<std::byte*>(ptr) -
                    reinterpret_cast<std::byte*>(header);
    uint32_t slot = (offset - kPageConfig.slots_start_offset) / kItemSize;
    header->occupancy[slot / 64] |= (uint64_t{1} << (slot % 64));
  }

  // If enabled, create a key with the current generation.
  if constexpr (EnableGenerations) {
    // Map slot index to key.
    // We use index + 1 because keys interpret 0 as null/invalid.
    return {ptr, PoolAllocatorKey(index + 1, generations_[index])};
  } else {
    return ptr;
  }
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
void PoolAllocator<T, EnableGenerations, MaxBlockPower>::Deallocate(T* obj) {
  if (!obj) {
    return;
  }

  deallocation_generation_++;

  // Destruct the object, must be called explicitly when using placement new.
  obj->~T();

  uint32_t index = ~0u;

  // 1. Check for item in Small Blocks
  // We can do this as a linear scan, because there are very few small blocks
  // it's very fast.
  bool found_small = false;
  uint32_t small_block_start_index = 0;
  for (size_t i = 0; i < small_blocks_.size(); ++i) {
    SmallBlock& block = small_blocks_[i];
    // Check if the object's address falls within the memory range of this
    // small block.
    if (reinterpret_cast<std::byte*>(obj) >= block.start &&
        reinterpret_cast<std::byte*>(obj) < block.end) {
      size_t offset = reinterpret_cast<std::byte*>(obj) - block.start;
      
      index = small_block_start_index + (offset / kItemSize);
      small_block_pointers_[index] = nullptr;
      found_small = true;
      break;
    }
    small_block_start_index += block.count;
  }

  if (!found_small) {
    // 2. Check Pages via Masking
    // We can find the page mask from the pointer due to power of 2 page size.
    // PageHeader contains start_index.
    PageHeader* page_header = GetPageHeader(obj);

    uint32_t start_index = page_header->start_index;
    

    size_t offset = reinterpret_cast<std::byte*>(obj) -
                    reinterpret_cast<std::byte*>(page_header);
    size_t slot = (offset - kPageConfig.slots_start_offset) / kItemSize;

    
    

    index = start_index + slot;

    // Update page occupancy
    page_header->occupancy[slot / 64] &= ~(uint64_t{1} << (slot % 64));
  }

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
  Slot* slot = reinterpret_cast<Slot*>(obj);
  slot->free_slot.next_free_ptr = nullptr;
  slot->free_slot.index = index;

  if (free_tail_ptr_) {
    free_tail_ptr_->free_slot.next_free_ptr = slot;
    free_tail_ptr_ = slot;
  } else {
    // List was empty
    free_head_ptr_ = slot;
    free_tail_ptr_ = slot;
  }
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
T* PoolAllocator<T, EnableGenerations, MaxBlockPower>::ResolveKey(
    PoolAllocatorKey key) const {
  static_assert(EnableGenerations);

  uint32_t slot = key.GetSlot();

  // This simultaneously tests that the slot is non-empty and  within range
  // using a single check. For an empty key, GetSlot() returns UINT32_MAX, which
  // will fail the capacity check. The max number of slots is lower than
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
    const SmallBlock& block = small_blocks_[block_idx];
    uint32_t start_index = (uint32_t{1} << block_idx) - 1;
    uint32_t offset_idx = slot - start_index;
    return reinterpret_cast<T*>(block.start + offset_idx * kItemSize);
  } else {
    uint32_t relative_idx = slot - kSmallBlocksCapacity;
    uint32_t page_idx = relative_idx / kPageConfig.slots_per_page;
    uint32_t slot_idx = relative_idx % kPageConfig.slots_per_page;
    std::byte* page_mem = reinterpret_cast<std::byte*>(pages_[page_idx]);
    std::byte* slots_start = page_mem + kPageConfig.slots_start_offset;
    return reinterpret_cast<T*>(slots_start + slot_idx * kItemSize);
  }
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
bool PoolAllocator<T, EnableGenerations, MaxBlockPower>::IsKeyValid(
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

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
uint32_t PoolAllocator<T, EnableGenerations, MaxBlockPower>::GetAllocatedCount()
    const {
  return allocated_count_;
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
uint32_t PoolAllocator<T, EnableGenerations, MaxBlockPower>::GetCapacity()
    const {
  return current_capacity_;
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
constexpr size_t
PoolAllocator<T, EnableGenerations, MaxBlockPower>::GetItemSize() {
  return kItemSize;
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
constexpr
    typename PoolAllocator<T, EnableGenerations, MaxBlockPower>::PageConfig
    PoolAllocator<T, EnableGenerations, MaxBlockPower>::ComputePageConfig() {
  // Target number of slots in a page.
  // The real number of slots could be higher or lower depending on which power
  // of 2 page size is picked and header overhead.
  size_t soft_target_count = std::max(128, 1 << kMaxBlockPower);

  // Bytes to hold the soft target number of slots, ignoring the header.
  size_t soft_target_bytes = kItemSize * soft_target_count;

  // Pick the smallest power of 2 page size that can hold the soft target
  // number of slots, up to 64KB.
  size_t page_size_bytes = 4096;
  while (page_size_bytes < soft_target_bytes && page_size_bytes < 65536) {
    page_size_bytes *= 2;
  }

  // The number of slots that would fit in the page if there were no header.
  size_t num_potential_slots = page_size_bytes / kItemSize;

  // The number of 64-bit words needed for the occupancy bitmap to cover all of
  // the potential slots.
  size_t bitmap_words = (num_potential_slots + 63) / 64;

  // Now we need to find how much space the header will take up.
  // It consists of a start_index (uint32_t), then the occupancy bitmap

  // The alignment between start_index and occupancy.
  size_t occupancy_alignment = alignof(uint64_t);

  // Offset between start_index and occupancy, accounting for alignment.
  size_t bitmap_offset =
      (sizeof(uint32_t) + occupancy_alignment - 1) & ~(occupancy_alignment - 1);

  // The number of bytes required for the bitmap, which is the number of words *
  // 8 because each word is 64 bits.
  size_t bitmap_bytes = bitmap_words * 8;

  // The total size of the header.
  size_t header_overhead = bitmap_offset + bitmap_bytes;

  // The alignment requirement of Slot, used to determine where the slots start
  // after the header.
  size_t slot_alignment = alignof(Slot);

  // The offset from the start of the header to the first slot, correctly
  // accounting for memory alignment.
  size_t slots_start_offset =
      (header_overhead + slot_alignment - 1) & ~(slot_alignment - 1);

  // The actual number of slots that fit in the page after the header size is
  // accounted for.
  size_t slots_per_page = (page_size_bytes - slots_start_offset) / kItemSize;

  return {.page_size_bytes = static_cast<uint32_t>(page_size_bytes),
          .slots_per_page = static_cast<uint32_t>(slots_per_page),
          .bitmap_words = static_cast<uint32_t>(bitmap_words),
          .slots_start_offset = static_cast<uint32_t>(slots_start_offset)};
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
void PoolAllocator<T, EnableGenerations, MaxBlockPower>::AddSmallBlock() {
  uint32_t block_idx = small_blocks_.size();
  uint32_t count = uint32_t{1} << block_idx;
  uint32_t bytes = count * kItemSize;

  std::byte* mem;

  // If T is an over-aligned type, we must use aligned new to ensure the
  // memory block adheres to T's alignment requirements. Otherwise,
  // standard new[] is sufficient.
  if constexpr (alignof(T) > alignof(std::max_align_t)) {
    mem = static_cast<std::byte*>(
        ::operator new[](bytes, std::align_val_t{alignof(T)}, std::nothrow));
  } else {
    mem = new (std::nothrow) std::byte[bytes];
  }

  cursor_ptr_ = reinterpret_cast<T*>(mem);
  small_blocks_.push_back({mem, mem + bytes, count});
  current_capacity_ += count;

  // Note: We do not need to resize generations_ here because we make space
  // for kSmallBlocksCapacity elements in the constructor.
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
void PoolAllocator<T, EnableGenerations, MaxBlockPower>::AddPage() {
  void* mem = ::operator new(kPageConfig.page_size_bytes,
                             std::align_val_t{kPageConfig.page_size_bytes},
                             std::nothrow);

  uint32_t start_idx = GetCapacity();

  PageHeader* header = new (mem) PageHeader{start_idx, {}};

  pages_.push_back(header);
  current_capacity_ += kPageConfig.slots_per_page;

  cursor_ptr_ = reinterpret_cast<T*>(reinterpret_cast<std::byte*>(header) +
                                     kPageConfig.slots_start_offset);

  if constexpr (EnableGenerations) {
    generations_.resize(current_capacity_, 0);
  }
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
template <typename Fn>
void PoolAllocator<T, EnableGenerations, MaxBlockPower>::ForEach(Fn&& fn) {
  iteration_depth_++;

  uint32_t end_index = cursor_index_;

  // Capture pages_count at the start in case a page is added during iteration.
  size_t pages_count = pages_.size();

  if (end_index > 0) {
    uint32_t sb_limit = std::min(end_index, kSmallBlocksCapacity);
    for (uint32_t i = 0; i < sb_limit; ++i) {
      if (small_block_pointers_[i]) {
        fn(small_block_pointers_[i]);
      }
    }
  }

  if (end_index > kSmallBlocksCapacity) {
    uint32_t captured_deallocation_generation = deallocation_generation_;
    uint32_t pages_end_index = end_index - kSmallBlocksCapacity;

    for (size_t page_index = 0; page_index < pages_count; ++page_index) {
      PageHeader* page_header = pages_[page_index];

      // Calculate limit based on end_index.
      uint32_t remaining_slots_to_check =
          pages_end_index - (page_index * kPageConfig.slots_per_page);

      // We only care about the limit within this page.
      uint32_t slots_in_page_to_check =
          std::min(remaining_slots_to_check, kPageConfig.slots_per_page);

      // It should not be possible for words_to_check to be zero here, because
      // the captured end_index and pages_count already limit the iteration.
      uint32_t words_to_check = (slots_in_page_to_check + 63) / 64;

      // Copy the occupancy bitmap into a local variable to scan through.
      // This allows us to mask off any bits that are beyond the slots to check,
      // which is important to avoid bugs when Allocate is called during
      // iteration.
      //
      // This could also be done by masking the incoming word within the loop,
      // but this performs better because it avoids a branch misprediction
      // inside the inner loop, and the copy is just a few uint64_t on the stack
      // per page which is practically free.
      std::array<uint64_t, kPageConfig.bitmap_words> occupancy_snapshot =
          page_header->occupancy;

      // Mask last word if needed.
      if (slots_in_page_to_check % 64 != 0) {
        uint32_t last_word_idx = words_to_check - 1;
        uint64_t mask = (uint64_t{1} << (slots_in_page_to_check % 64)) - 1;
        occupancy_snapshot[last_word_idx] &= mask;
      }

      Slot* page_base =
          reinterpret_cast<Slot*>(reinterpret_cast<std::byte*>(page_header) +
                                  kPageConfig.slots_start_offset);

      for (uint32_t w = 0; w < words_to_check; ++w) {
        uint64_t word = occupancy_snapshot[w];
        Slot* word_base = page_base + (w * 64);

        // Use countr_zero to scan through the occupied slots and process them.
        while (word != 0) {
          int bit_index = absl::countr_zero(word);

          // Fast path: no deallocations have occurred during iteration.
          // This check is predictable for the CPU so the branch is cheap.
          if (ABSL_PREDICT_TRUE(captured_deallocation_generation ==
                                deallocation_generation_)) {
            fn(&word_base[bit_index].object);
          } else {
            // A deallocation has occurred during iteration, re-check occupancy.
            // Intentionally nested so the compiler can generate more optimized
            // code for the fast path.
            if (page_header->occupancy[w] & (uint64_t{1} << bit_index)) {
              fn(&word_base[bit_index].object);
            }
          }

          // Clear the lowest set bit
          word &= (word - 1);
        }
      }
    }
  }

  iteration_depth_--;
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
PoolAllocator<T, EnableGenerations, MaxBlockPower>::PageHeader*
PoolAllocator<T, EnableGenerations, MaxBlockPower>::GetPageHeader(
    T* ptr) const {
  return reinterpret_cast<PageHeader*>(
      reinterpret_cast<uintptr_t>(ptr) &
      ~(static_cast<uintptr_t>(kPageConfig.page_size_bytes) - 1));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_POOL_ALLOCATOR_H_
