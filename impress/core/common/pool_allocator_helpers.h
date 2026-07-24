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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_POOL_ALLOCATOR_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_POOL_ALLOCATOR_HELPERS_H_

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace imp {

class PoolAllocatorKey;

namespace imp_pool_allocator_internal {

// Stores memory layout information for the allocator.
//
// PoolAllocator::ComputeMemoryLayout() is used to compute this struct at
// compile time for a given T. In some places, we use this information at
// compile time to optimize operations. In others, we store it and use it at
// runtime with type erasure to avoid code bloat.
struct MemoryLayout {
  // The size of the slot in bytes.
  size_t slot_size_bytes;
  // The alignment of the slot.
  size_t slot_alignment;
  // Power of 2 that configures the number of slots per page.
  uint32_t page_slots_power;
  // The number of slots in each page.
  uint32_t slots_per_page;
  // A mask of the page slot indices, used to quickly get the slot index within
  // a page.
  uint32_t page_slot_mask;
  // The number of 64-bit words needed to store the bitmap for each subset.
  uint32_t page_words_per_subset;
  // The offset from the start of the page to the footer.
  // The footer contains the bitmaps for each subset.
  uint32_t footer_offset_bytes;
};

// FreeSlot stores both it's own index and the pointer to the next free slot.
// This allows us to manage the free list with zero overhead to calculate the
// index or the pointer.
struct FreeSlot {
  FreeSlot* next_free_ptr;
  // This is the overall index of the slot in the pool across all small blocks
  // and pages. This is used by the Allocate function to quickly find the
  // index of the slot from the slot pointer. This saves CPU usage so we don't
  // need to find/calculate it.
  uint32_t index;
};

// A union of the object and the FreeSlot. This is used to allow us to store
// either the object or the FreeSlot in the same memory location.
//
// This allows us to manage the free list with zero memory overhead.
template <typename T>
union Slot {
  T object;
  imp_pool_allocator_internal::FreeSlot free_slot;
};

// Ensure the FreeSlot is trivially destructible because we skip calling
// destructors for it in the allocator.
static_assert(std::is_trivially_destructible_v<FreeSlot>);

// A contiguous range of slots that is smaller than the page size.
struct SmallBlock {
  std::byte* start;
  std::byte* end;
  uint32_t count;
};

namespace internal {

template <typename T,
          std::enable_if_t<std::is_same_v<PoolAllocatorKey,
                                          decltype(std::declval<const T&>()
                                                       .GetPoolAllocatorKey())>,
                           int> = 0>
static constexpr bool HasGetPoolKeyFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasGetPoolKeyFunc(...) {
  return false;
}

template <
    typename T,
    std::enable_if_t<
        std::is_same_v<void, decltype(std::declval<T&>().SetPoolAllocatorKey(
                                 std::declval<PoolAllocatorKey>()))>,
        int> = 0>
static constexpr bool HasSetPoolKeyFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasSetPoolKeyFunc(...) {
  return false;
}

}  // namespace internal

template <typename T>
static constexpr bool kHasGetPoolKey = internal::HasGetPoolKeyFunc<T>(0);

template <typename T>
static constexpr bool kHasSetPoolKey = internal::HasSetPoolKeyFunc<T>(0);

constexpr uint32_t GetPageWordsPerSubset(uint32_t page_slots_power) {
  return page_slots_power >= 6 ? (1 << (page_slots_power - 6)) : 1;
}

// Computes the size of the footer for a page with the given power and number of
// subsets.
constexpr size_t GetFooterSize(uint32_t page_slots_power, size_t num_subsets) {
  // The number of 64-bit words needed to store the bitmaps for each subset.
  // If page_slots_power < 6, then we have <= 64 slots per page, so we only need
  // 1 word per subset.
  uint32_t page_words = GetPageWordsPerSubset(page_slots_power);
  return (1 + num_subsets) * page_words * sizeof(uint64_t);
}

// Computes the size rounded up to the nearest multiple of slot_alignment.
//
// This is used because when allocating memory the size must be a multiple of
// the alignment.
constexpr size_t GetAlignedBytes(size_t bytes, size_t alignment) {
  return (bytes + alignment - 1) & ~(alignment - 1);
}

// Computes the size of a page in bytes accounting for alignment requirements.
constexpr size_t GetPageSize(const MemoryLayout& memory_layout,
                             size_t num_subsets) {
  size_t footer_size =
      GetFooterSize(memory_layout.page_slots_power, num_subsets);
  size_t page_size =
      memory_layout.slots_per_page * memory_layout.slot_size_bytes +
      footer_size;
  return memory_layout.slot_alignment > __STDCPP_DEFAULT_NEW_ALIGNMENT__
             ? GetAlignedBytes(page_size, memory_layout.slot_alignment)
             : page_size;
}

// Computes the page layout information for the allocator based on the type T.
template <typename T, uint32_t PageSlotsPower>
constexpr MemoryLayout ComputeMemoryLayout() {
  using Slot = Slot<T>;
  constexpr uint32_t slots_per_page = 1 << PageSlotsPower;
  constexpr uint32_t page_slot_mask = slots_per_page - 1;
  constexpr uint32_t page_words_per_subset =
      GetPageWordsPerSubset(PageSlotsPower);
  constexpr uint32_t footer_offset_bytes = slots_per_page * sizeof(Slot);

  return {
      .slot_size_bytes = sizeof(Slot),
      .slot_alignment = alignof(Slot),
      .page_slots_power = PageSlotsPower,
      .slots_per_page = slots_per_page,
      .page_slot_mask = page_slot_mask,
      .page_words_per_subset = page_words_per_subset,
      .footer_offset_bytes = footer_offset_bytes,
  };
}

// Returns true if the page size can be allocated within the given limit.
template <typename T, uint32_t Power>
constexpr bool PageFitsInLimit(size_t num_subsets, size_t limit) {
  // Allocating an object includes some memory overhead. That means that if we
  // allocate exactly 32kb bytes, the overhead will actually push us over the
  // limit into the next size class. Accounting for this overhead in this check
  // prevents that from happening.
  constexpr uint32_t kPageOverhead = 32;

  size_t page_size = GetPageSize(ComputeMemoryLayout<T, Power>(), num_subsets);
  return (page_size + kPageOverhead) <= limit;
}

template <typename T>
constexpr uint32_t GetPageSlotsPowerForType(size_t num_subsets) {
  // For very small objects, use a page size of 256 slots.
  // Target: under 32 KB page size.
  if (PageFitsInLimit<T, 8>(num_subsets, 32768)) {
    return 8;  // 256 slots
  }

  // For medium objects (the most common case).
  // Target: under 64 KB page size.
  if (PageFitsInLimit<T, 7>(num_subsets, 65536)) {
    return 7;  // 128 slots
  }

  // For larger objects, use a page size of 64 slots.
  // Target: under 64 KB page size.
  if (PageFitsInLimit<T, 6>(num_subsets, 65536)) {
    return 6;  // 64 slots
  }

  // For very large objects, use a page size of 32 slots.
  // Target: under 64 KB page size.
  if (PageFitsInLimit<T, 5>(num_subsets, 65536)) {
    return 5;  // 32 slots
  }

  // Fallback case for 16 slots per page.
  return 4;  // 16 slot per page
}

}  // namespace imp_pool_allocator_internal

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_POOL_ALLOCATOR_HELPERS_H_
