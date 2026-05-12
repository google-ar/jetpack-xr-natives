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

#include "core/performance/memory_stats.h"

#include <atomic>
#include <cstddef>

#include "core/performance/android/editor_malloc.h"

namespace imp {

namespace {

struct ThreadData {
  size_t allocated_bytes;
  size_t allocations_count;
};

// Thread-specific key for storing the thread-local data.
static pthread_key_t thread_key;

// Initialize the thread-specific key on first use.
static pthread_once_t key_once = PTHREAD_ONCE_INIT;

// Uses the real free function to avoid ours.
static void free_thread_data(void* data) { imp_malloc::Free(data); }

// Create a thread-specific key and provide our custom destructor.
static void make_key() { pthread_key_create(&thread_key, free_thread_data); }

// Retrieves the allocation frame data for the current thread.
// This function is over-commented as it is a bit confusing otherwise.
ThreadData* GetThreadData() {
  // Create the thread key on first use.
  pthread_once(&key_once, make_key);

  // Get the thread-specific data stored under the key.
  ThreadData* data = static_cast<ThreadData*>(pthread_getspecific(thread_key));

  // If the data is set, return it.
  if (data) return data;

  // If the data is not set, allocate it.
  // We exposed the real malloc function to avoid a stackoverflow here.
  data = static_cast<ThreadData*>(imp_malloc::Malloc(sizeof(ThreadData)));

  // Allocation failed, return nullptr.
  if (!data) return nullptr;

  data->allocated_bytes = 0;
  data->allocations_count = 0;

  // Sets the thread-specific data under the key.
  pthread_setspecific(thread_key, data);

  return data;
}

}  // namespace

// The other platforms use thread_local variables for counters.
// On Android using thread_local variables allocates causing a stackoverflow.
// When the thread_local counter is first accessed it calls malloc which
// calls back into IncrementMemoryCounters which adds to the thread_local
// variable which calls malloc which calls ...
// To avoid this issue we use pthread_key to store the thread specific data.

size_t MemoryStats::GetMemoryBytesAllocatedOnThisThread() {
  ThreadData* thread_data = GetThreadData();

  if (!thread_data) return 0;

  return thread_data->allocated_bytes;
}

size_t MemoryStats::GetAllocationsCountOnThisThread() {
  ThreadData* thread_data = GetThreadData();

  if (!thread_data) return 0;

  return thread_data->allocations_count;
}

void MemoryStats::ResetMemoryCountersForThisThread() {
  ThreadData* thread_data = GetThreadData();
  if (thread_data) {
    thread_data->allocated_bytes = 0;
    thread_data->allocations_count = 0;
  }
}

size_t MemoryStats::GetMemoryUsageBytes() {
  return allocated_bytes_total_.load(std::memory_order_relaxed);
}

size_t MemoryStats::GetAllocationsCountTotal() {
  return allocations_count_total_.load(std::memory_order_relaxed);
}

void MemoryStats::IncrementMemoryCounters(size_t size) {
  ThreadData* thread_data = GetThreadData();
  if (thread_data) {
    thread_data->allocated_bytes += size;
    thread_data->allocations_count += 1;
  }
  allocated_bytes_total_.fetch_add(size, std::memory_order_relaxed);
  allocations_count_total_.fetch_add(1, std::memory_order_relaxed);
}

void MemoryStats::DecrementMemoryCounters(size_t size) {
  allocated_bytes_total_.fetch_sub(size, std::memory_order_relaxed);
  allocations_count_total_.fetch_sub(1, std::memory_order_relaxed);
}
}  // namespace imp
