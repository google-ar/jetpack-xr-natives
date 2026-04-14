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
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#include "absl/base/optimization.h"
#include "absl/numeric/bits.h"
#include "core/common/base_pool_allocator.h"
#include "core/common/pool_allocator_helpers.h"

namespace imp {

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
class PoolAllocator
    : public BasePoolAllocator<EnableGenerations, MaxBlockPower> {
 public:
  using Base = BasePoolAllocator<EnableGenerations, MaxBlockPower>;

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

  // Allocates and constructs an object of type T.

  template <typename... Args>
  AllocateResult Allocate(Args&&... args);

  // Destructs and deallocates an object of type T.
  void Deallocate(T* obj);

  // Resolves a key to a pointer. Returns nullptr if the key is invalid or
  // stale (generation mismatch).
  T* ResolveKey(PoolAllocatorKey key) const;

  // Returns the size of the slots stored by this allocator in bytes. Not
  // guaranteed to be the same as sizeof(T) if T is smaller than 12 bytes.
  static constexpr size_t GetSlotSize();

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
  using MemoryLayout = imp_pool_allocator_internal::MemoryLayout;

  // When Deallocate is called, we use the memory of the slot to intrusively
  // store both the index and pointer of the next free slot in the free list.
  // This allows us to manage the free list with zero lookup overhead.
  union Slot {
    T object;
    imp_pool_allocator_internal::FreeSlot free_slot;
  };

  // Computes the page layout information based on the type T.
  static constexpr MemoryLayout ComputeMemoryLayout();
};

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
PoolAllocator<T, EnableGenerations, MaxBlockPower>::PoolAllocator()
    : Base(ComputeMemoryLayout()) {
  // Ensure T's alignment is respected.
  static_assert(sizeof(Slot) + ComputeMemoryLayout().slots_start_offset_bytes <=
                    ComputeMemoryLayout().page_size_bytes,
                "T is too large for PoolAllocator pages");
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
template <typename... Args>
typename PoolAllocator<T, EnableGenerations, MaxBlockPower>::AllocateResult
PoolAllocator<T, EnableGenerations, MaxBlockPower>::Allocate(Args&&... args) {
  uint32_t index;
  T* ptr = reinterpret_cast<T*>(Base::ClaimSlot(&index));

  // Construct object using placement new within the slot.
  new (ptr) T(std::forward<Args>(args)...);

  // Track that the slot is occupied. This is done outside of ClaimSlot to allow
  // the compiler to optimize based on the compile-time known memory layout.
  if (index < Base::kSmallBlocksCapacity) {
    Base::small_block_pointers_[index] = ptr;
  } else {
    constexpr MemoryLayout kMemoryLayout = ComputeMemoryLayout();
    imp_pool_allocator_internal::SetOccupancy(
        reinterpret_cast<std::byte*>(ptr),
        ~(static_cast<uintptr_t>(kMemoryLayout.page_size_bytes) - 1),
        GetSlotSize(), kMemoryLayout.slots_start_offset_bytes);
  }

  // If enabled, create a key with the current generation.
  if constexpr (EnableGenerations) {
    // Map slot index to key.
    // We use index + 1 because keys interpret 0 as null/invalid.
    return {ptr, PoolAllocatorKey(index + 1, Base::generations_[index])};
  } else {
    return ptr;
  }
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
void PoolAllocator<T, EnableGenerations, MaxBlockPower>::Deallocate(T* obj) {
  if (!obj) {
    return;
  }

  // Destruct the object, must be called explicitly when using placement new.
  obj->~T();

  constexpr MemoryLayout kMemoryLayout = ComputeMemoryLayout();

  uint32_t index;
  uint32_t small_block_start_index;
  std::byte* small_block_start_ptr;

  if (Base::FindInSmallBlock(reinterpret_cast<std::byte*>(obj),
                             &small_block_start_index,
                             &small_block_start_ptr)) {
    size_t offset = reinterpret_cast<std::byte*>(obj) - small_block_start_ptr;
    index = small_block_start_index + (offset / GetSlotSize());
    Base::small_block_pointers_[index] = nullptr;
  } else {
    // Track that the slot is unoccupied. This is done outside of ReleaseSlot to
    // allow the compiler to optimize based on the compile-time known memory
    // layout.
    index = imp_pool_allocator_internal::UnsetOccupancy(
        reinterpret_cast<std::byte*>(obj),
        ~(static_cast<uintptr_t>(kMemoryLayout.page_size_bytes) - 1),
        GetSlotSize(), kMemoryLayout.slots_start_offset_bytes);
  }

  Base::ReleaseSlot(obj, index);
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
T* PoolAllocator<T, EnableGenerations, MaxBlockPower>::ResolveKey(
    PoolAllocatorKey key) const {
  return reinterpret_cast<T*>(Base::GetSlotPointer(key));
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
constexpr size_t
PoolAllocator<T, EnableGenerations, MaxBlockPower>::GetSlotSize() {
  return sizeof(Slot);
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
constexpr imp_pool_allocator_internal::MemoryLayout
PoolAllocator<T, EnableGenerations, MaxBlockPower>::ComputeMemoryLayout() {
  size_t slot_size_bytes = sizeof(Slot);

  // Target number of slots in a page.
  // The real number of slots could be higher or lower depending on which power
  // of 2 page size is picked and header overhead.
  size_t soft_target_count = std::max(128, 1 << MaxBlockPower);

  // Bytes to hold the soft target number of slots, ignoring the header.
  size_t soft_target_bytes = slot_size_bytes * soft_target_count;

  // Pick the smallest power of 2 page size that can hold the soft target
  // number of slots, up to 64KB.
  size_t page_size_bytes = 4096;
  while (page_size_bytes < soft_target_bytes && page_size_bytes < 65536) {
    page_size_bytes *= 2;
  }

  // The number of slots that would fit in the page if there were no header.
  size_t num_potential_slots = page_size_bytes / slot_size_bytes;

  // The number of 64-bit words needed for the occupancy bitmap to cover all of
  // the potential slots.
  size_t bitmap_words_per_page = (num_potential_slots + 63) / 64;

  // Now we need to find how much space the header will take up.
  // It consists of a start_index (uint32_t), then the occupancy bitmap

  // The alignment between start_index and occupancy.
  size_t occupancy_alignment = alignof(uint64_t);

  // Offset between start_index and occupancy, accounting for alignment.
  size_t bitmap_offset =
      (sizeof(uint32_t) + occupancy_alignment - 1) & ~(occupancy_alignment - 1);

  // The number of bytes required for the bitmap, which is the number of words *
  // 8 because each word is 64 bits.
  size_t bitmap_bytes = bitmap_words_per_page * 8;

  // The total size of the header.
  size_t header_overhead = bitmap_offset + bitmap_bytes;

  // The alignment requirement of Slot, used to determine where the slots start
  // after the header.
  size_t slot_alignment = alignof(Slot);

  // The offset from the start of the header to the first slot, correctly
  // accounting for memory alignment.
  size_t slots_start_offset_bytes =
      (header_overhead + slot_alignment - 1) & ~(slot_alignment - 1);

  // The actual number of slots that fit in the page after the header size is
  // accounted for.
  size_t slots_per_page =
      (page_size_bytes - slots_start_offset_bytes) / slot_size_bytes;

  return {.slot_size_bytes = slot_size_bytes,
          .slot_alignment = slot_alignment,
          .page_size_bytes = static_cast<uint32_t>(page_size_bytes),
          .slots_per_page = static_cast<uint32_t>(slots_per_page),
          .bitmap_words_per_page = static_cast<uint32_t>(bitmap_words_per_page),
          .slots_start_offset_bytes =
              static_cast<uint32_t>(slots_start_offset_bytes)};
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower>
template <typename Fn>
void PoolAllocator<T, EnableGenerations, MaxBlockPower>::ForEach(Fn&& fn) {
  // Use the memory layout computed at compile time instead of the one stored on
  // the object so that the values can be inlined for performance.
  constexpr MemoryLayout kMemoryLayout = ComputeMemoryLayout();

  Base::iteration_depth_++;

  // Captured at the start of ForEach so we don't iterate over objects allocated
  // during iteration.
  uint32_t end_index = Base::cursor_index_;
  size_t pages_count = Base::pages_.size();

  if (end_index > 0) {
    uint32_t sb_limit = std::min(end_index, Base::kSmallBlocksCapacity);
    for (uint32_t i = 0; i < sb_limit; ++i) {
      if (Base::small_block_pointers_[i]) {
        fn(reinterpret_cast<T*>(Base::small_block_pointers_[i]));
      }
    }
  }

  if (end_index > Base::kSmallBlocksCapacity) {
    uint32_t captured_deallocation_generation = Base::deallocation_generation_;

    const size_t last_page_index = pages_count - 1;
    const uint32_t slots_in_final_page =
        end_index - Base::kSmallBlocksCapacity -
        (last_page_index * kMemoryLayout.slots_per_page);
    uint32_t remainder = slots_in_final_page % 64;
    const uint64_t last_word_mask =
        remainder == 0 ? ~uint64_t{0} : (uint64_t{1} << remainder) - 1;
    const uint32_t last_word_index = ((slots_in_final_page + 63) / 64) - 1;

    for (size_t page_index = 0; page_index < pages_count; ++page_index) {
      std::byte* page_base = Base::pages_[page_index];
      uint64_t* occupancy =
          imp_pool_allocator_internal::GetPageOccupancy(page_base);

      const uint32_t last_word_index_for_page =
          page_index == last_page_index
              ? last_word_index
              : kMemoryLayout.bitmap_words_per_page - 1;

      Slot* page_slot_base = reinterpret_cast<Slot*>(
          page_base + kMemoryLayout.slots_start_offset_bytes);

      for (uint32_t w = 0; w <= last_word_index_for_page; ++w) {
        uint64_t mask =
            (page_index == last_page_index && w == last_word_index_for_page)
                ? last_word_mask
                : ~uint64_t{0};
        uint64_t word = occupancy[w] & mask;
        Slot* slot_base = page_slot_base + (w * 64);

        // Use countr_zero to scan through the occupied slots and process them.
        while (word != 0) {
          int bit_index = absl::countr_zero(word);

          // Fast path: no deallocations have occurred during iteration.
          // This check is predictable for the CPU so the branch is cheap.
          if (ABSL_PREDICT_TRUE(captured_deallocation_generation ==
                                Base::deallocation_generation_)) {
            fn(&slot_base[bit_index].object);
          } else {
            // A deallocation has occurred during iteration, re-check occupancy.
            // Intentionally nested so the compiler can generate more optimized
            // code for the fast path.
            if (occupancy[w] & (uint64_t{1} << bit_index)) {
              fn(&slot_base[bit_index].object);
            }
          }

          // Clear the lowest set bit
          word &= (word - 1);
        }
      }
    }
  }

  Base::iteration_depth_--;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_POOL_ALLOCATOR_H_
