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

#include <cstddef>
#include <cstdint>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/base/optimization.h"
#include "absl/log/check.h"
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
// Low Capacity: Geometric Small Blocks
//   - For the first few allocations (indices `0-14` by default, configured via
//   `MaxBlockPower`), memory is allocated in geometrically growing blocks
//   (sizes 1, 2, 4, 8).
//
// High Capacity: Fixed-size Pages
//  - Once the "Small Block" capacity is reached, the allocator switches to
//    pages that contain a fixed number of slots. The number of slots per page
//    is always a power of 2. By default, it's determined based on the size of T
//    but can be specified via `PageSlotsPower`.
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
// NumSubsets configures the number of parallel subset bitmaps to maintain
// for efficient sparse iteration. When NumSubsets > 0, objects can be
// associated with subsets using SetSubsetBit, and iterated via
// ForEach<SubsetIdx>.
//
// Key Features:
// - Pointer stability
// - O(1) Allocate (Intrusive FreeList).
// - O(1) Deallocate (Page Masking).
// - O(1) ResolveKey (Fast key validation).
// - Subset tracking for efficient sparse iteration.
// - Minimal memory waste for small element counts.
// - Zero memory overhead for per-item headers.
//
// API Contract:
// - The type T must implement a method:
//     `PoolAllocatorKey GetPoolAllocatorKey() const`
//   which returns the key associated with the object.
// - Optionally, T can implement a method:
//     `void SetPoolAllocatorKey(PoolAllocatorKey key)`
//   which will be automatically called by the allocator during `Allocate` to
//   store the key.
template <typename T, bool EnableGenerations = true, uint32_t MaxBlockPower = 4,
          size_t NumSubsets = 0,
          uint32_t PageSlotsPower =
              imp_pool_allocator_internal::GetPageSlotsPowerForType<T>(
                  NumSubsets)>
class PoolAllocator
    : public BasePoolAllocator<EnableGenerations, MaxBlockPower, NumSubsets> {
  static_assert(
      imp_pool_allocator_internal::kHasGetPoolKey<T>,
      "T must implement GetPoolAllocatorKey() returning PoolAllocatorKey");

 public:
  using Base = BasePoolAllocator<EnableGenerations, MaxBlockPower, NumSubsets>;
  using MemoryLayout = imp_pool_allocator_internal::MemoryLayout;

  struct PtrAndKey {
    // The pointer to the allocated object.
    T* ptr;

    // If EnableGenerations is true, this key can be passed into ResolveKey to
    // lookup ptr later. If ptr has been deallocated, then ResolveKey will
    // return nullptr.
    PoolAllocatorKey key;
  };

  using AllocateResult = PtrAndKey;

  static constexpr MemoryLayout kMemoryLayout =
      imp_pool_allocator_internal::ComputeMemoryLayout<T, PageSlotsPower>();

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
  void ForEach(Fn&& fn) {
    ForEach<Base::kOccupiedSubset>(std::forward<Fn>(fn));
  }

  // Iterates over all allocated objects in the given subset and calls the given
  // function for each object.
  template <size_t SubsetIdx, typename Fn>
  void ForEach(Fn&& fn);

 private:
  using Slot = imp_pool_allocator_internal::Slot<T>;

  // Helper function to call the given function if the object is still in the
  // subset and is not null.
  //
  // This function is used for the cold path in ForEach. Marked as noinline to
  // keep the hot path cold path branch prediction simple and predictable.
  template <typename Fn>
  ABSL_ATTRIBUTE_NOINLINE void CallIfStillInSubsetSmallBlock(
      size_t internal_subset_idx, int bit_index, Fn&& fn);

  // Helper function to call the given function if the object is still in the
  // subset and is not null.
  //
  // This function is used for the cold path in ForEach. Marked as noinline to
  // keep the hot path cold path branch prediction simple and predictable.
  template <typename Fn>
  ABSL_ATTRIBUTE_NOINLINE void CallIfStillInSubsetPage(
      uint64_t* subset_bitmap, uint32_t w, int bit_index, Slot* slot_base,
      Slot* slot_limit, Fn&& fn);

  // Returns a pointer to the page bitmaps for the given page base pointer.
  //
  // The page bitmaps are stored in the footer of the page starting from the
  // first word in the occupancy bitmap.
  //
  // This is static because the layout is compile-time known (`kMemoryLayout`),
  // allowing the compiler to optimize the hot path by folding the offset.
  // For the runtime code-size-optimized fallback layout helper, see
  // `BasePoolAllocator::GetPageBitmapsWithDynamicLayout`.
  static uint64_t* GetPageBitmapsWithStaticLayout(std::byte* page_base);

  static_assert(Base::kSmallBlocksCapacity < 64,
                "MaxBlockPower is too large for subset tracking");
};

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower,
          size_t NumSubsets, uint32_t PageSlotsPower>
PoolAllocator<T, EnableGenerations, MaxBlockPower, NumSubsets,
              PageSlotsPower>::PoolAllocator()
    : Base(kMemoryLayout) {
  static_assert(imp_pool_allocator_internal::PageFitsInLimit<T, PageSlotsPower>(
                    NumSubsets, 65536),
                "Page size must fit within 64KB");
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower,
          size_t NumSubsets, uint32_t PageSlotsPower>
template <typename... Args>
typename PoolAllocator<T, EnableGenerations, MaxBlockPower, NumSubsets,
                       PageSlotsPower>::AllocateResult
PoolAllocator<T, EnableGenerations, MaxBlockPower, NumSubsets,
              PageSlotsPower>::Allocate(Args&&... args) {
  uint32_t index;
  T* ptr = reinterpret_cast<T*>(Base::ClaimSlot(&index));

  // Construct object using placement new within the slot.
  new (ptr) T(std::forward<Args>(args)...);

  // Track that the slot is occupied. This is done outside of ClaimSlot to allow
  // the compiler to optimize based on the compile-time known memory layout.
  if (index < Base::kSmallBlocksCapacity) {
    Base::small_block_pointers_[index] = ptr;
    Base::small_block_subsets_[0] |= (1ULL << index);
  } else {
    uint32_t relative_idx = index - Base::kSmallBlocksCapacity;
    uint32_t page_idx = relative_idx >> PageSlotsPower;
    uint32_t slot_idx = relative_idx & kMemoryLayout.page_slot_mask;
    std::byte* page_base = Base::pages_[page_idx];
    uint64_t* bitmaps = GetPageBitmapsWithStaticLayout(page_base);
    bitmaps[slot_idx / 64] |= (1ULL << (slot_idx % 64));
  }

  uint16_t generation = 0;
  if constexpr (EnableGenerations) {
    generation = Base::generations_[index];
  }
  PoolAllocatorKey key(index, generation);
  if constexpr (imp_pool_allocator_internal::kHasSetPoolKey<T>) {
    ptr->SetPoolAllocatorKey(key);
  }
  return {ptr, key};
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower,
          size_t NumSubsets, uint32_t PageSlotsPower>
void PoolAllocator<T, EnableGenerations, MaxBlockPower, NumSubsets,
                   PageSlotsPower>::Deallocate(T* obj) {
  if (!obj) {
    return;
  }

  PoolAllocatorKey key = obj->GetPoolAllocatorKey();

  // Destruct the object, must be called explicitly when using placement new.
  obj->~T();

  
  uint32_t index = key.GetSlot();

  if (index < Base::kSmallBlocksCapacity) {
    
    Base::small_block_pointers_[index] = nullptr;
    for (size_t i = 0; i <= NumSubsets; ++i) {
      Base::small_block_subsets_[i] &= ~(1ULL << index);
    }
  } else {
    // Remove the slot from the subset bitmaps. This is done outside of
    // ReleaseSlot to allow the compiler to optimize based on the compile-time
    // known memory layout.
    uint32_t relative_idx = index - Base::kSmallBlocksCapacity;
    uint32_t page_idx = relative_idx >> PageSlotsPower;
    uint32_t slot_idx = relative_idx & kMemoryLayout.page_slot_mask;
    size_t word_idx = slot_idx / 64;
    uint64_t bit = ~(1ULL << (slot_idx % 64));

    std::byte* page_base = Base::pages_[page_idx];
    uint64_t* bitmaps = GetPageBitmapsWithStaticLayout(page_base);

    for (size_t i = 0; i <= NumSubsets; ++i) {
      bitmaps[i * kMemoryLayout.page_words_per_subset + word_idx] &= bit;
    }
  }

  Base::ReleaseSlot(obj, index);
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower,
          size_t NumSubsets, uint32_t PageSlotsPower>
T* PoolAllocator<T, EnableGenerations, MaxBlockPower, NumSubsets,
                 PageSlotsPower>::ResolveKey(PoolAllocatorKey key) const {
  return reinterpret_cast<T*>(Base::GetSlotPointer(key));
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower,
          size_t NumSubsets, uint32_t PageSlotsPower>
constexpr size_t PoolAllocator<T, EnableGenerations, MaxBlockPower, NumSubsets,
                               PageSlotsPower>::GetSlotSize() {
  return sizeof(Slot);
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower,
          size_t NumSubsets, uint32_t PageSlotsPower>
template <size_t SubsetIdx, typename Fn>
void PoolAllocator<T, EnableGenerations, MaxBlockPower, NumSubsets,
                   PageSlotsPower>::ForEach(Fn&& fn) {
  static_assert(SubsetIdx == Base::kOccupiedSubset || SubsetIdx < NumSubsets,
                "Subset index out of bounds");
  constexpr size_t kInternalSubsetIdx =
      (SubsetIdx == Base::kOccupiedSubset) ? 0 : 1 + SubsetIdx;

  Base::iteration_depth_++;

  uint32_t end_index = Base::cursor_index_;
  uint16_t pages_count = Base::pages_count_;
  uint32_t captured_generation = Base::modification_generation_;

  if (end_index > 0) {
    uint64_t word = Base::small_block_subsets_[kInternalSubsetIdx];
    while (word != 0) {
      // Use countr_zero to scan through the slots and process them.
      int bit_index = absl::countr_zero(word);

      // Fast path: no modifications have occurred during iteration.
      // This check is predictable for the CPU so the branch is cheap.
      if (ABSL_PREDICT_TRUE(captured_generation ==
                            Base::modification_generation_)) {
        fn(reinterpret_cast<T*>(Base::small_block_pointers_[bit_index]));
      } else {
        // A modification has occurred during iteration, re-check bitmap.
        // Intentionally nested so the compiler can generate more optimized
        // code for the fast path.
        CallIfStillInSubsetSmallBlock(kInternalSubsetIdx, bit_index, fn);
      }

      // Clear the lowest set bit
      word &= (word - 1);
    }
  }

  if (end_index > Base::kSmallBlocksCapacity) {
    const size_t last_page_index = pages_count - 1;
    const uint32_t slots_in_final_page =
        end_index - Base::kSmallBlocksCapacity -
        (last_page_index * kMemoryLayout.slots_per_page);

    for (size_t page_index = 0; page_index < pages_count; ++page_index) {
      std::byte* page_base = Base::pages_[page_index];

      uint64_t* subset_bitmap =
          GetPageBitmapsWithStaticLayout(page_base) +
          kInternalSubsetIdx * kMemoryLayout.page_words_per_subset;

      Slot* slot_limit =
          reinterpret_cast<Slot*>(page_base) +
          (page_index == last_page_index ? slots_in_final_page
                                         : kMemoryLayout.slots_per_page);

      for (uint32_t w = 0; w < kMemoryLayout.page_words_per_subset; ++w) {
        Slot* slots = reinterpret_cast<Slot*>(page_base) + (w * 64);
        uint64_t word = subset_bitmap[w];

        // Use countr_zero to scan through the slots and process them.
        while (word != 0) {
          int bit_index = absl::countr_zero(word);

          // Fast path: no modifications have occurred during iteration.
          // This check is predictable for the CPU so the branch is cheap.
          if (ABSL_PREDICT_TRUE(captured_generation ==
                                Base::modification_generation_)) {
            fn(&slots[bit_index].object);
          } else {
            // A modification has occurred during iteration, re-check bitmap.
            // Intentionally nested so the compiler can generate more optimized
            // code for the fast path.
            CallIfStillInSubsetPage(subset_bitmap, w, bit_index, slots,
                                    slot_limit, fn);
          }

          // Clear the lowest set bit
          word &= (word - 1);
        }
      }
    }
  }

  Base::iteration_depth_--;
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower,
          size_t NumSubsets, uint32_t PageSlotsPower>
template <typename Fn>
ABSL_ATTRIBUTE_NOINLINE void PoolAllocator<
    T, EnableGenerations, MaxBlockPower, NumSubsets,
    PageSlotsPower>::CallIfStillInSubsetSmallBlock(size_t internal_subset_idx,
                                                   int bit_index, Fn&& fn) {
  bool is_still_in_subset = (Base::small_block_subsets_[internal_subset_idx] &
                             (1ULL << bit_index)) != 0;
  if (is_still_in_subset && Base::small_block_pointers_[bit_index] != nullptr) {
    fn(reinterpret_cast<T*>(Base::small_block_pointers_[bit_index]));
  }
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower,
          size_t NumSubsets, uint32_t PageSlotsPower>
template <typename Fn>
ABSL_ATTRIBUTE_NOINLINE void
PoolAllocator<T, EnableGenerations, MaxBlockPower, NumSubsets,
              PageSlotsPower>::CallIfStillInSubsetPage(uint64_t* subset_bitmap,
                                                       uint32_t w,
                                                       int bit_index,
                                                       Slot* slot_base,
                                                       Slot* slot_limit,
                                                       Fn&& fn) {
  Slot* slot_ptr = &slot_base[bit_index];
  if (slot_ptr < slot_limit && (subset_bitmap[w] & (1ULL << bit_index))) {
    fn(&slot_ptr->object);
  }
}

template <typename T, bool EnableGenerations, uint32_t MaxBlockPower,
          size_t NumSubsets, uint32_t PageSlotsPower>
uint64_t* PoolAllocator<
    T, EnableGenerations, MaxBlockPower, NumSubsets,
    PageSlotsPower>::GetPageBitmapsWithStaticLayout(std::byte* page_base) {
  return reinterpret_cast<uint64_t*>(page_base +
                                     kMemoryLayout.footer_offset_bytes);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_POOL_ALLOCATOR_H_
