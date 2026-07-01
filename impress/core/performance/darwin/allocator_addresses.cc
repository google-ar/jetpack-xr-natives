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

#include "core/performance/allocator_addresses.h"

#include <dlfcn.h>
#include <malloc/malloc.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>

// There is an absl macro for this but it would add a BUILD select for one item.
#if __cplusplus >= 201703L || defined(__cpp_noexcept_function_type)
#define NOEXCEPT_TYPE noexcept
#else
#define NOEXCEPT_TYPE
#endif

extern "C" {
void* malloc(size_t);
void* calloc(size_t, size_t);
void* realloc(void*, size_t);
void* aligned_alloc(size_t, size_t);
int posix_memalign(void**, size_t, size_t);
void* reallocf(void* ptr, size_t size);
void* memalign(size_t alignment, size_t size);
void* valloc(size_t size);
void* pvalloc(size_t size);
void* malloc_zone_malloc(malloc_zone_t* zone, size_t size);
void* malloc_zone_calloc(malloc_zone_t* zone, size_t nmemb, size_t size);
void* malloc_zone_realloc(malloc_zone_t* zone, void* ptr, size_t size);
void* malloc_zone_memalign(malloc_zone_t* zone, size_t alignment, size_t size);
void* malloc_zone_valloc(malloc_zone_t* zone, size_t size);
}

namespace imp {

namespace {

// Helper to get the address of an allocator function.
// Names are short for readability to keep most of array entries under 80 chars.
template <typename T>
const void* Alloc(T func) {
  return reinterpret_cast<const void*>(func);
}

// Helper to get the address of an overloaded operator new/delete.
template <typename Signature>
const void* New(Signature* func) {
  return reinterpret_cast<const void*>(func);
}

// List of all the functions that we consider to be allocators.
// Because we perform a linear search, the most common functions should be at
// the start of the list for highest probability of a quick find.
static const void* kAllocatorAddresses[] = {
    New<void*(size_t)>(&::operator new),
    New<void*(size_t)>(&::operator new[]),
    Alloc(malloc),
    Alloc(calloc),
    Alloc(realloc),

#ifdef __cpp_aligned_new
    // Common: Aligned C++ allocators
    New<void*(size_t, std::align_val_t)>(&::operator new),
    New<void*(size_t, std::align_val_t)>(&::operator new[]),
#endif

    // Common: Darwin zone allocators
    Alloc(malloc_zone_malloc),
    Alloc(malloc_zone_calloc),
    Alloc(malloc_zone_realloc),

    // Occasional: Nothrow C++ allocators
    New<void*(size_t, const std::nothrow_t&)NOEXCEPT_TYPE>(&::operator new),
    New<void*(size_t, const std::nothrow_t&)NOEXCEPT_TYPE>(&::operator new[]),

#ifdef __cpp_aligned_new
    New<void*(size_t, std::align_val_t, const std::nothrow_t&)NOEXCEPT_TYPE>(
        &::operator new),
    New<void*(size_t, std::align_val_t, const std::nothrow_t&)NOEXCEPT_TYPE>(
        &::operator new[]),
#endif

    // Occasional: Aligned C allocators
    Alloc(aligned_alloc),
    Alloc(posix_memalign),
    Alloc(malloc_zone_memalign),

    // Rare / Legacy
    Alloc(reallocf),
    Alloc(memalign),
    Alloc(valloc),
    Alloc(pvalloc),
    Alloc(malloc_zone_valloc),
};

// Number of addresses we've stored in the allocator array.
static constexpr size_t kAllocatorAddressesCount =
    sizeof(kAllocatorAddresses) / sizeof(kAllocatorAddresses[0]);

// Maximum fallback window size (in bytes) to search for an address inside
// an allocator function if dynamic lookup (dladdr) fails or is in progress.
static constexpr size_t kFallbackFunctionSize = 128;

// Maximum budget (in bytes) to scan forward from the function start address
// looking for a symbol boundary.
static constexpr size_t kMaxScanRangeBytes = 2048;

// Start and end addresses for each allocator function.
struct AllocatorRange {
  uintptr_t start;
  uintptr_t end;
};

// Ranges of addresses for each allocator function.
static AllocatorRange kAllocatorRanges[kAllocatorAddressesCount];

// Initialization state for the allocator ranges.
enum class InitState {
  kUninitialized,
  kInitializing,
  kInitialized,
};

// Thread-safe initialization state for the allocator ranges.
static std::atomic<InitState> g_init_state{InitState::kUninitialized};

// Initialize the allocator ranges with fallback first to handle any re-entrant
// allocations during the dladdr lookup scan.
void InitializeAllocatorRanges() {
  // Query dladdr to compute precise end addresses for each allocator.
  for (size_t i = 0; i < kAllocatorAddressesCount; ++i) {
    Dl_info info;
    const void* base = kAllocatorAddresses[i];
    const uintptr_t base_addr =
        reinterpret_cast<uintptr_t>(kAllocatorAddresses[i]);

    kAllocatorRanges[i].start = base_addr;
    kAllocatorRanges[i].end = base_addr + kFallbackFunctionSize;

    // If the address was not found or the symbol doesn't match, skip.
    // The symbol may not always match due to compiler optimizations.
    if (!dladdr(base, &info) || info.dli_saddr != base) continue;

    // Scan ahead to find the boundary of the symbol or mapping.
    for (size_t offset = 1; offset < kMaxScanRangeBytes; ++offset) {
      Dl_info next_info;
      const void* next_addr = reinterpret_cast<const void*>(base_addr + offset);
      const bool found_address = dladdr(next_addr, &next_info);

      // If the address was found and the symbol matches, continue scanning.
      if (found_address && next_info.dli_saddr == base) continue;

      // If the address was not found we hit a mapping boundary.
      // If the symbol didn't match we hit a function boundary.
      // In either case, we found the end of the function.
      kAllocatorRanges[i].end = base_addr + offset;

      break;
    }
  }
}

// Checks whether the address falls within any of the known allocator ranges.
inline bool IsInAnyAllocator(uintptr_t target) {
  for (size_t i = 0; i < kAllocatorAddressesCount; ++i) {
    if (target >= kAllocatorRanges[i].start &&
        target < kAllocatorRanges[i].end) {
      return true;
    }
  }

  return false;
}

// Checks if the address is in the window of base addr + kFallbackFunctionSize.
inline bool IsInFallbackWindow(uintptr_t target) {
  for (size_t i = 0; i < kAllocatorAddressesCount; ++i) {
    const uintptr_t base = reinterpret_cast<uintptr_t>(kAllocatorAddresses[i]);

    if (target >= base && target < base + kFallbackFunctionSize) return true;
  }

  return false;
}

}  // namespace

// We're using dladdr once at startup to resolve the actual boundaries of each
// allocator function. If initialization fails or is in progress, we fall back
// to a heuristic search window.
bool IsAllocatorAddress(const void* addr) {
  const uintptr_t target = reinterpret_cast<uintptr_t>(addr);

  InitState state = g_init_state.load(std::memory_order_acquire);

  if (state == InitState::kUninitialized) {
    InitState expected = InitState::kUninitialized;

    // Check that the state is still uninitialized to prevent race conditions.
    if (g_init_state.compare_exchange_strong(expected, InitState::kInitializing,
                                             std::memory_order_relaxed)) {
      InitializeAllocatorRanges();
      g_init_state.store(InitState::kInitialized, std::memory_order_release);
      state = InitState::kInitialized;
    }
  }

  if (state == InitState::kInitialized) {
    return IsInAnyAllocator(target);
  }
  return IsInFallbackWindow(target);
}

}  // namespace imp
