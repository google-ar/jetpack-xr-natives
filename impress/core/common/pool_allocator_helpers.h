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

namespace imp {

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
  // The size of each page in bytes.
  uint32_t page_size_bytes;
  // The number of slots in each page.
  uint32_t slots_per_page;
  // The number of 64-bit words needed to store the bitmap storing which slots
  // are occupied.
  uint32_t bitmap_words_per_page;
  // The offset from the start of the page to the footer.
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
// Ensure the FreeSlot is trivially destructible because we skip calling
// destructors for it in the allocator.
static_assert(std::is_trivially_destructible_v<FreeSlot>);

// A contiguous range of slots that is smaller than the page size.
struct SmallBlock {
  std::byte* start;
  std::byte* end;
  uint32_t count;
};

// Information stored at the end of each page.
struct PageFooter {
  uint32_t start_index;
};
// Ensure the PageFooter is trivially destructible because we skip calling
// destructors for it in the allocator.
static_assert(std::is_trivially_destructible_v<PageFooter>);

// Returns a pointer to the occupancy bitmap for the given page.
inline uint64_t* GetPageOccupancy(std::byte* page_base,
                                  uint32_t footer_offset_bytes) {
  constexpr size_t kOccupancyAlignment = alignof(uint64_t);
  constexpr size_t kBitmapOffset =
      (sizeof(imp_pool_allocator_internal::PageFooter) + kOccupancyAlignment -
       1) &
      ~(kOccupancyAlignment - 1);
  return reinterpret_cast<uint64_t*>(page_base + footer_offset_bytes +
                                     kBitmapOffset);
}

// Sets the occupancy bit for the given slot.
inline void SetOccupancy(std::byte* ptr, uintptr_t page_mask, size_t slot_size,
                         uint32_t footer_offset_bytes) {
  std::byte* page_base = reinterpret_cast<std::byte*>(
      reinterpret_cast<uintptr_t>(ptr) & page_mask);
  size_t slot = (ptr - page_base) / slot_size;
  GetPageOccupancy(page_base, footer_offset_bytes)[slot / 64] |=
      (uint64_t{1} << (slot % 64));
}

// Unsets the occupancy bit for the given slot.
inline uint32_t UnsetOccupancy(std::byte* ptr, uintptr_t page_mask,
                               size_t slot_size, uint32_t footer_offset_bytes) {
  std::byte* page_base = reinterpret_cast<std::byte*>(
      reinterpret_cast<uintptr_t>(ptr) & page_mask);
  size_t slot = (ptr - page_base) / slot_size;
  GetPageOccupancy(page_base, footer_offset_bytes)[slot / 64] &=
      ~(uint64_t{1} << (slot % 64));
  using PageFooter = imp_pool_allocator_internal::PageFooter;
  return reinterpret_cast<PageFooter*>(page_base + footer_offset_bytes)
             ->start_index +
         slot;
}

}  // namespace imp_pool_allocator_internal

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_POOL_ALLOCATOR_HELPERS_H_
