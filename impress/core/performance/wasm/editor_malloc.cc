// Copyright 2024 Google LLC
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

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

#include "absl/log/check.h"
#include "core/performance/memory_stats.h"

// Declarations for the real functions
extern "C" {

void* __real_malloc(size_t size);
void __real_free(void* ptr);
void* __real_calloc(size_t nmemb, size_t size);
void* __real_realloc(void* ptr, size_t size);
void* __real_aligned_alloc(size_t alignment, size_t size);
int __real_posix_memalign(void** memptr, size_t alignment, size_t size);
void* __real_memalign(size_t alignment, size_t size);
}

namespace {

// For this platform, we wrap the system versions of each malloc/free variant to
// add a small header to each allocation. This header contains the size of the
// allocation so that we can report it to the profiler.
// It also contains a magic number so that we can identify the allocation as one
// that we wrapped. Allocations that bypass our wrapper such as Filament's
// custom allocator will not have this header or magic number and will be
// ignored by the profiler.

// WASM supports linker options __wrap_{function} in order to replace
// all calls to {function} with calls to __wrap_{function}. It also provides
// __real_{function} which is the original function.

constexpr uint32_t kMagicNumber = 0xFEEDADAD;
// This magic number is used to identify freed allocations.
// Using DEADBEEF as this is a convention for freed memory.
constexpr uint32_t kFreedMagicNumber = 0xDEADBEEF;

// Header to store the size of the allocation
struct AllocHeader {
  size_t size;
  uint32_t magic;
};

// Advance pointer by the size of the header to get the wrapped pointer.
inline void* UserPtrFromRaw(void* raw_ptr) {
  return static_cast<void*>(static_cast<char*>(raw_ptr) + sizeof(AllocHeader));
}

// Subtract pointer by the size of the header to get the raw pointer that
// includes the header.
inline void* RawPtrFromUser(void* user_ptr) {
  return static_cast<void*>(static_cast<char*>(user_ptr) - sizeof(AllocHeader));
}
}  // namespace

extern "C" {

void* __wrap_malloc(size_t size) {
  size_t total_size = size + sizeof(AllocHeader);
  void* raw_ptr = __real_malloc(total_size);
  if (!raw_ptr) return nullptr;

  AllocHeader* header = static_cast<AllocHeader*>(raw_ptr);
  header->size = size;
  header->magic = kMagicNumber;

  imp::MemoryStats::Get().IncrementMemoryCounters(size);
  return UserPtrFromRaw(raw_ptr);
}

void __wrap_free(void* ptr) {
  if (!ptr) return;

  AllocHeader* header = reinterpret_cast<AllocHeader*>(RawPtrFromUser(ptr));

  if (header->magic == kMagicNumber) {
    size_t size = header->size;
    imp::MemoryStats::Get().DecrementMemoryCounters(size);
    header->magic = kFreedMagicNumber;
    __real_free(header);
  } else if (header->magic == kFreedMagicNumber) {
    // IMP_LOG(imp::WARNING) << "Double-free of pointer: " << ptr;
  } else {
    // This case happens if the allocation came from a custom allocator that
    // bypassed our custom malloc. e.g. Filament uses a custom allocator for
    // certain things that calls mmap directly rather than malloc.
    // That memory is freed however with free() so it would decrement our
    // counters without ever having been incremented in the first place.
    // Leads to the memory graph being incorrect and goes negative over time.
    __real_free(ptr);
  }
}

void* __wrap_calloc(size_t nmemb, size_t size) {
  size_t total_user_size = nmemb * size;
  size_t total_alloc_size = total_user_size + sizeof(AllocHeader);
  void* raw_ptr = __real_malloc(total_alloc_size);

  if (!raw_ptr) return nullptr;

  AllocHeader* header = static_cast<AllocHeader*>(raw_ptr);
  header->size = total_user_size;
  header->magic = kMagicNumber;

  void* user_ptr = UserPtrFromRaw(raw_ptr);
  memset(user_ptr, 0, total_user_size);

  imp::MemoryStats::Get().IncrementMemoryCounters(total_user_size);
  return user_ptr;
}

void* __wrap_realloc(void* ptr, size_t size) {
  if (!ptr) return __wrap_malloc(size);

  AllocHeader* header = reinterpret_cast<AllocHeader*>(RawPtrFromUser(ptr));

  if (header->magic != kMagicNumber) {
    // Not our allocation, delegate to __real_realloc directly
    return __real_realloc(ptr, size);
  }

  if (size == 0) {
    __wrap_free(ptr);
    return nullptr;
  }

  size_t old_size = header->size;
  size_t total_new_size = size + sizeof(AllocHeader);
  void* raw_new_ptr = __real_realloc(header, total_new_size);

  if (!raw_new_ptr) return nullptr;

  AllocHeader* new_header = static_cast<AllocHeader*>(raw_new_ptr);
  new_header->size = size;
  new_header->magic = kMagicNumber;

  imp::MemoryStats::Get().DecrementMemoryCounters(old_size);
  imp::MemoryStats::Get().IncrementMemoryCounters(size);

  return UserPtrFromRaw(raw_new_ptr);
}

void* __wrap_aligned_alloc(size_t alignment, size_t size) {
  return __real_memalign(alignment, size);
}

// Obsolete functions.
int __wrap_posix_memalign(void** memptr, size_t alignment, size_t size) {
  return __real_posix_memalign(memptr, alignment, size);
}

void __wrap_cfree(void* ptr) { __wrap_free(ptr); }

void* __wrap_memalign(size_t alignment, size_t size) {
  return __real_memalign(alignment, size);
}

void* __wrap_valloc(size_t size) {
  // Emulate valloc by aligning to page size (commonly 4k)
  constexpr size_t page_size = 4096;
  return __wrap_memalign(page_size, size);
}

void* __wrap_pvalloc(size_t size) {
  constexpr size_t page_size = 4096;
  size_t rounded_size = (size + page_size - 1) & ~(page_size - 1);
  return __wrap_memalign(page_size, rounded_size);
}

}  // extern "C"

void* operator new(size_t size) {
  void* ptr = __wrap_malloc(size);
  
  return ptr;
}

void* operator new[](size_t size) {
  void* ptr = __wrap_malloc(size);
  
  return ptr;
}

void* operator new(size_t size, const std::nothrow_t&) noexcept {
  return __wrap_malloc(size);
}

void* operator new[](size_t size, const std::nothrow_t&) noexcept {
  return __wrap_malloc(size);
}

void operator delete(void* ptr) noexcept { __wrap_free(ptr); }

void operator delete[](void* ptr) noexcept { __wrap_free(ptr); }

void operator delete(void* ptr, const std::nothrow_t&) noexcept {
  __wrap_free(ptr);
}

void operator delete[](void* ptr, const std::nothrow_t&) noexcept {
  __wrap_free(ptr);
}

void operator delete(void* ptr, size_t size) noexcept { __wrap_free(ptr); }

void operator delete[](void* ptr, size_t size) noexcept { __wrap_free(ptr); }
