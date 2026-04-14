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

#include <dlfcn.h>
#include <malloc/malloc.h>
#include <pthread.h>
#include <unistd.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

#include "absl/base/const_init.h"
#include "absl/base/optimization.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/log/check.h"
#include "absl/synchronization/mutex.h"
#include "core/performance/memory_stats.h"

// For this platform, we do not need to add a header to each allocation since
// MacOS supports malloc_size which returns the size of the allocation.

// We also cannot use the header technique as MacOS already provides and expects
// its own header to be present for all allocations when compiler optimizations
// are enabled. Attempting to add our own header causes issues with the system
// level allocator functions.

// MacOS does NOT support linker options __wrap_{function} but provides a path
// to the original functions using dlsym(RTLD_NEXT, "function_name").
// This library has to be linked before any others during a build in order to
// override these functions.

// Without a header we need to track the allocations to know which originated
// from our malloc override. These means we need to track all allocations
// using a set. The set used for this has a custom allocator that uses the
// original malloc to avoid an infinite loop. Unfortunately to keep this
// thread-safe we need to use a mutex.

namespace {
static void* (*real_malloc_unsafe)(size_t) = nullptr;
static void (*real_free_unsafe)(void*) = nullptr;
static void* (*real_calloc_unsafe)(size_t, size_t) = nullptr;
static void* (*real_realloc_unsafe)(void*, size_t) = nullptr;
static void* (*real_reallocf_unsafe)(void*, size_t) = nullptr;
static void* (*real_aligned_alloc_unsafe)(size_t, size_t) = nullptr;
static int (*real_posix_memalign_unsafe)(void**, size_t, size_t) = nullptr;
static void (*real_cfree_unsafe)(void*) = nullptr;
static void* (*real_memalign_unsafe)(size_t, size_t) = nullptr;
static void* (*real_valloc_unsafe)(size_t) = nullptr;
static void* (*real_pvalloc_unsafe)(size_t) = nullptr;

// Malloc Zone function types
static void* (*real_malloc_zone_malloc_unsafe)(malloc_zone_t*,
                                               size_t) = nullptr;
static void* (*real_malloc_zone_calloc_unsafe)(malloc_zone_t*, size_t,
                                               size_t) = nullptr;
static void* (*real_malloc_zone_realloc_unsafe)(malloc_zone_t*, void*,
                                                size_t) = nullptr;
static void (*real_malloc_zone_free_unsafe)(malloc_zone_t*, void*) = nullptr;
static void* (*real_malloc_zone_memalign_unsafe)(malloc_zone_t*, size_t,
                                                 size_t) = nullptr;
static void* (*real_malloc_zone_valloc_unsafe)(malloc_zone_t*,
                                               size_t) = nullptr;

template <typename T>
void LoadSymbol(T& func_ptr, const char* name) {
  func_ptr = reinterpret_cast<T>(dlsym(RTLD_NEXT, name));
  if (func_ptr == nullptr) {
    // Do not use LOG here as it allocates memory and will crash.
    fprintf(stderr,
            "Failed to find allocator function: %s. Memory tracking may be "
            "less accurate.\n",
            name);
  }
}

// Custom Allocator using real_malloc/real_free to avoid an infinite loop.
// If we track all of our allocations in a set, the set itself allocates memory
// for each element which would cause an infinite loop without this allocator.
template <class T>
struct RealMallocAllocator {
  using value_type = T;

  RealMallocAllocator() = default;
  template <class U>
  constexpr RealMallocAllocator(const RealMallocAllocator<U>&) noexcept {}

  [[nodiscard]] T* allocate(size_t n) {
    // No safety check to see if real_malloc_unsafe is ready to avoid a branch.
    
    void* p = real_malloc_unsafe(n * sizeof(T));
    
    return static_cast<T*>(p);
  }

  void deallocate(T* p, size_t) noexcept {
    // No safety check to see if real_free_unsafe is ready to avoid a branch.
    real_free_unsafe(p);
  }
};

template <class T, class U>
bool operator==(const RealMallocAllocator<T>&, const RealMallocAllocator<U>&) {
  return true;
}
template <class T, class U>
bool operator!=(const RealMallocAllocator<T>&, const RealMallocAllocator<U>&) {
  return false;
}

using VoidPtrSet =
    absl::flat_hash_set<void*, absl::Hash<void*>, std::equal_to<void*>,
                        RealMallocAllocator<void*>>;
static VoidPtrSet* active_allocations = nullptr;

static absl::Mutex alloc_mutex(absl::kConstInit);

static pthread_once_t init_once = PTHREAD_ONCE_INIT;

static void Init() {
  LoadSymbol(real_malloc_unsafe, "malloc");
  LoadSymbol(real_free_unsafe, "free");
  
  
  LoadSymbol(real_calloc_unsafe, "calloc");
  LoadSymbol(real_realloc_unsafe, "realloc");
  LoadSymbol(real_reallocf_unsafe, "reallocf");
  LoadSymbol(real_aligned_alloc_unsafe, "aligned_alloc");
  LoadSymbol(real_posix_memalign_unsafe, "posix_memalign");
  LoadSymbol(real_cfree_unsafe, "cfree");
  LoadSymbol(real_memalign_unsafe, "memalign");
  LoadSymbol(real_valloc_unsafe, "valloc");
  LoadSymbol(real_pvalloc_unsafe, "pvalloc");

  LoadSymbol(real_malloc_zone_malloc_unsafe, "malloc_zone_malloc");
  LoadSymbol(real_malloc_zone_calloc_unsafe, "malloc_zone_calloc");
  LoadSymbol(real_malloc_zone_realloc_unsafe, "malloc_zone_realloc");
  LoadSymbol(real_malloc_zone_free_unsafe, "malloc_zone_free");
  LoadSymbol(real_malloc_zone_memalign_unsafe, "malloc_zone_memalign");
  LoadSymbol(real_malloc_zone_valloc_unsafe, "malloc_zone_valloc");

  // Only need to track allocations if the memory graph is supported.
  if (!imp::MemoryStats::IsMemoryGraphSupported()) return;

  // Set's allocator is dependent on the real_ functions.
  void* set_mem = real_malloc_unsafe(sizeof(VoidPtrSet));
  
  active_allocations =
      new (set_mem) VoidPtrSet(0, absl::Hash<void*>(), std::equal_to<void*>(),
                               RealMallocAllocator<void*>());
}

template <typename Func>
inline Func GetRealFunc(Func& func_unsafe) {
  pthread_once(&init_once, Init);
  
  return func_unsafe;
}

// Helper to add a pointer to the tracked set
inline void TrackAllocation(void* ptr) {
  if (ABSL_PREDICT_FALSE(!ptr)) return;

  if (imp::MemoryStats::IsMemoryGraphSupported()) {
    absl::MutexLock lock(&alloc_mutex);
    active_allocations->insert(ptr);
  }

  imp::MemoryStats::Get().IncrementMemoryCounters(malloc_size(ptr));
}

// Helper to remove a pointer and return true if it was tracked
inline void UntrackAllocation(void* ptr) {
  if (!imp::MemoryStats::IsMemoryGraphSupported()) return;

  bool removed = false;
  {
    absl::MutexLock lock(&alloc_mutex);
    removed = active_allocations->erase(ptr) > 0;
  }

  if (ABSL_PREDICT_FALSE(!removed)) return;

  size_t size = malloc_size(ptr);
  imp::MemoryStats::Get().DecrementMemoryCounters(size);
}
}  // namespace

extern "C" {
void* malloc(size_t size) {
  void* ptr = GetRealFunc(real_malloc_unsafe)(size);
  TrackAllocation(ptr);
  return ptr;
}

void free(void* ptr) {
  if (ABSL_PREDICT_FALSE(!ptr)) return;
  UntrackAllocation(ptr);
  GetRealFunc(real_free_unsafe)(ptr);
}

void* calloc(size_t nmemb, size_t size) {
  void* ptr = GetRealFunc(real_calloc_unsafe)(nmemb, size);
  TrackAllocation(ptr);
  return ptr;
}

void* realloc(void* ptr, size_t size) {
  if (!imp::MemoryStats::IsMemoryGraphSupported()) {
    void* new_ptr = GetRealFunc(real_realloc_unsafe)(ptr, size);
    imp::MemoryStats::Get().IncrementMemoryCounters(malloc_size(new_ptr));
    return new_ptr;
  }

  if (ABSL_PREDICT_FALSE(!ptr)) return malloc(size);

  size_t old_size = malloc_size(ptr);

  if (ABSL_PREDICT_FALSE(size == 0)) {
    free(ptr);  // free() handles decrementing the counter.
    return nullptr;
  }

  bool tracked;
  {
    absl::MutexLock lock(&alloc_mutex);
    tracked = active_allocations->count(ptr);
  }

  void* new_ptr = GetRealFunc(real_realloc_unsafe)(ptr, size);

  if (ABSL_PREDICT_TRUE(new_ptr)) {
    if (ABSL_PREDICT_TRUE(tracked)) {
      imp::MemoryStats::Get().DecrementMemoryCounters(old_size);
      imp::MemoryStats::Get().IncrementMemoryCounters(malloc_size(new_ptr));

      absl::MutexLock lock(&alloc_mutex);
      active_allocations->erase(ptr);
      active_allocations->insert(new_ptr);
    }
  }
  // If new_ptr is null, real_realloc failed. ptr is still valid,
  // so no change to memory counters is needed for the old block.
  return new_ptr;
}

void* reallocf(void* ptr, size_t size) {
  if (!imp::MemoryStats::IsMemoryGraphSupported()) {
    void* new_ptr = GetRealFunc(real_reallocf_unsafe)(ptr, size);
    imp::MemoryStats::Get().IncrementMemoryCounters(malloc_size(new_ptr));
    return new_ptr;
  }

  if (ABSL_PREDICT_FALSE(!ptr)) return malloc(size);

  size_t old_size = malloc_size(ptr);

  if (ABSL_PREDICT_FALSE(size == 0)) {
    free(ptr);  // free() handles decrementing the counter.
    return nullptr;
  }

  bool tracked;
  {
    absl::MutexLock lock(&alloc_mutex);
    tracked = active_allocations->count(ptr);
  }

  void* new_ptr = GetRealFunc(real_reallocf_unsafe)(ptr, size);

  if (ABSL_PREDICT_TRUE(new_ptr)) {
    if (ABSL_PREDICT_TRUE(tracked)) {
      imp::MemoryStats::Get().DecrementMemoryCounters(old_size);
      imp::MemoryStats::Get().IncrementMemoryCounters(malloc_size(new_ptr));

      absl::MutexLock lock(&alloc_mutex);
      active_allocations->erase(ptr);
      active_allocations->insert(new_ptr);
    }
  } else if (ABSL_PREDICT_TRUE(tracked)) {
    // reallocf failed, and the original ptr was freed by real_reallocf.
    imp::MemoryStats::Get().DecrementMemoryCounters(old_size);
    absl::MutexLock lock(&alloc_mutex);
    active_allocations->erase(ptr);
  }
  return new_ptr;
}

void* aligned_alloc(size_t alignment, size_t size) {
  void* ptr = GetRealFunc(real_aligned_alloc_unsafe)(alignment, size);
  TrackAllocation(ptr);
  return ptr;
}

int posix_memalign(void** memptr, size_t alignment, size_t size) {
  int result = GetRealFunc(real_posix_memalign_unsafe)(memptr, alignment, size);

  static constexpr int kPosixMemalignSuccess = 0;
  if (result == kPosixMemalignSuccess) TrackAllocation(*memptr);

  return result;
}

// Obsolete functions.
void cfree(void* ptr) {
  if (ABSL_PREDICT_FALSE(!ptr)) return;
  UntrackAllocation(ptr);
  GetRealFunc(real_cfree_unsafe)(ptr);
}

void* memalign(size_t alignment, size_t size) {
  void* ptr = GetRealFunc(real_memalign_unsafe)(alignment, size);
  TrackAllocation(ptr);
  return ptr;
}

void* valloc(size_t size) {
  void* ptr = GetRealFunc(real_valloc_unsafe)(size);
  TrackAllocation(ptr);
  return ptr;
}

void* pvalloc(size_t size) {
  void* ptr = GetRealFunc(real_pvalloc_unsafe)(size);
  TrackAllocation(ptr);
  return ptr;
}
}  // extern "C"

extern "C" {
// Zone versions
void* malloc_zone_malloc(malloc_zone_t* zone, size_t size) {
  void* ptr = GetRealFunc(real_malloc_zone_malloc_unsafe)(zone, size);
  TrackAllocation(ptr);
  return ptr;
}

void* malloc_zone_calloc(malloc_zone_t* zone, size_t nmemb, size_t size) {
  void* ptr = GetRealFunc(real_malloc_zone_calloc_unsafe)(zone, nmemb, size);
  TrackAllocation(ptr);
  return ptr;
}

void* malloc_zone_realloc(malloc_zone_t* zone, void* ptr, size_t size) {
  if (!imp::MemoryStats::IsMemoryGraphSupported()) {
    void* new_ptr =
        GetRealFunc(real_malloc_zone_realloc_unsafe)(zone, ptr, size);
    imp::MemoryStats::Get().IncrementMemoryCounters(malloc_size(new_ptr));
    return new_ptr;
  }

  if (!ptr) return malloc_zone_malloc(zone, size);

  size_t old_size = malloc_size(ptr);
  if (size == 0) {
    malloc_zone_free(zone, ptr);
    return nullptr;
  }

  bool tracked;
  {
    absl::MutexLock lock(&alloc_mutex);
    tracked = active_allocations->count(ptr);
  }

  void* new_ptr = GetRealFunc(real_malloc_zone_realloc_unsafe)(zone, ptr, size);

  if (new_ptr) {
    if (tracked) {
      imp::MemoryStats::Get().DecrementMemoryCounters(old_size);
      imp::MemoryStats::Get().IncrementMemoryCounters(malloc_size(new_ptr));

      absl::MutexLock lock(&alloc_mutex);
      active_allocations->erase(ptr);
      active_allocations->insert(new_ptr);
    }
  }
  return new_ptr;
}

void malloc_zone_free(malloc_zone_t* zone, void* ptr) {
  if (ABSL_PREDICT_FALSE(!ptr)) return;
  UntrackAllocation(ptr);
  GetRealFunc(real_malloc_zone_free_unsafe)(zone, ptr);
}

void* malloc_zone_memalign(malloc_zone_t* zone, size_t alignment, size_t size) {
  void* ptr =
      GetRealFunc(real_malloc_zone_memalign_unsafe)(zone, alignment, size);
  TrackAllocation(ptr);
  return ptr;
}

void* malloc_zone_valloc(malloc_zone_t* zone, size_t size) {
  void* ptr = GetRealFunc(real_malloc_zone_valloc_unsafe)(zone, size);
  TrackAllocation(ptr);
  return ptr;
}

}  // extern "C"

// Overload operator new and delete
void* operator new(size_t size) {
  void* ptr = malloc(size);
  return ptr;
}

void* operator new[](size_t size) { return malloc(size); }

void* operator new(size_t size, const std::nothrow_t&) noexcept {
  return malloc(size);
}

void* operator new[](size_t size, const std::nothrow_t&) noexcept {
  return malloc(size);
}

void operator delete(void* ptr) noexcept { free(ptr); }

void operator delete[](void* ptr) noexcept { free(ptr); }

void operator delete(void* ptr, size_t size) noexcept { free(ptr); }

void operator delete[](void* ptr, size_t size) noexcept { free(ptr); }

void operator delete(void* ptr, const std::nothrow_t&) noexcept { free(ptr); }

void operator delete[](void* ptr, const std::nothrow_t&) noexcept { free(ptr); }

#ifdef __cpp_aligned_new
void* operator new(size_t size, std::align_val_t al) {
  return aligned_alloc(static_cast<size_t>(al), size);
}
void* operator new[](size_t size, std::align_val_t al) {
  return aligned_alloc(static_cast<size_t>(al), size);
}
void* operator new(size_t size, std::align_val_t al,
                   const std::nothrow_t&) noexcept {
  return aligned_alloc(static_cast<size_t>(al), size);
}
void* operator new[](size_t size, std::align_val_t al,
                     const std::nothrow_t&) noexcept {
  return aligned_alloc(static_cast<size_t>(al), size);
}
void operator delete(void* ptr, std::align_val_t al) noexcept { free(ptr); }
void operator delete[](void* ptr, std::align_val_t al) noexcept { free(ptr); }
void operator delete(void* ptr, size_t size, std::align_val_t al) noexcept {
  free(ptr);
}
void operator delete[](void* ptr, size_t size, std::align_val_t al) noexcept {
  free(ptr);
}
void operator delete(void* ptr, std::align_val_t al,
                     const std::nothrow_t&) noexcept {
  free(ptr);
}
void operator delete[](void* ptr, std::align_val_t al,
                       const std::nothrow_t&) noexcept {
  free(ptr);
}
void operator delete(void* ptr, size_t size, std::align_val_t al,
                     const std::nothrow_t&) noexcept {
  free(ptr);
}
void operator delete[](void* ptr, size_t size, std::align_val_t al,
                       const std::nothrow_t&) noexcept {
  free(ptr);
}
#endif
