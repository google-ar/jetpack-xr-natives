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

#include <execinfo.h>

#include <algorithm>
#include <atomic>
#include <cstddef>

#include "absl/debugging/stacktrace.h"

namespace {
std::thread::id GetCachedThreadId() {
  thread_local const std::thread::id cached_id = std::this_thread::get_id();
  return cached_id;
}
}  // namespace

namespace imp {
namespace {
thread_local size_t allocated_bytes_by_thread_ = 0;
thread_local size_t allocations_count_by_thread_ = 0;
}  // namespace

size_t MemoryStats::GetMemoryBytesAllocatedOnThisThread() {
  return allocated_bytes_by_thread_;
}
size_t MemoryStats::GetAllocationsCountOnThisThread() {
  return allocations_count_by_thread_;
}
void MemoryStats::ResetMemoryCountersForThisThread() {
  allocated_bytes_by_thread_ = 0;
  allocations_count_by_thread_ = 0;
}

size_t MemoryStats::GetMemoryUsageBytes() {
  return allocated_bytes_total_.load(std::memory_order_relaxed);
}

size_t MemoryStats::GetAllocationsCountTotal() {
  return allocations_count_total_.load(std::memory_order_relaxed);
}

void MemoryStats::IncrementMemoryCounters(size_t size) {
  allocated_bytes_by_thread_ += size;
  allocations_count_by_thread_++;

  if (IsMemoryGraphSupported()) {
    allocated_bytes_total_.fetch_add(size, std::memory_order_relaxed);
    allocations_count_total_.fetch_add(1, std::memory_order_relaxed);
  }

  if (record_callstacks_.load(std::memory_order_relaxed)) {
    // Since callstack_index is uint32_t it will naturally wrap to 0 so we don't
    // need to worry about it overflowing. This is why the % is after the fetch.
    const int index = callstack_index_.fetch_add(1, std::memory_order_relaxed) %
                      kMaxCallstacks;
    Callstack& callstack_entry = callstacks_[index];

    // Skip 3 frames, TrackAllocation, IncrementMemoryCounters, and malloc.
    static constexpr int kSkippedStackFrames = 3;
    void* stack[kMaxCallstackDepth + kSkippedStackFrames];
    // MacOS backtrace is orders of magnitude faster than absl::GetStackTrace.
    const int depth =
        backtrace(stack, kMaxCallstackDepth + kSkippedStackFrames);
    const int frames_to_copy =
        std::max(0, std::min(depth - kSkippedStackFrames, kMaxCallstackDepth));
    callstack_entry.depth = frames_to_copy;
    // Copy the relevant stack frames into the callstack entry.
    memcpy(callstack_entry.callstack.data(), stack + kSkippedStackFrames,
           frames_to_copy * sizeof(void*));
    callstack_entry.size = size;
    callstack_entry.thread_id = GetCachedThreadId();
  }
}

void MemoryStats::DecrementMemoryCounters(size_t size) {
  if (IsMemoryGraphSupported()) {
    allocated_bytes_total_.fetch_sub(size, std::memory_order_relaxed);
    allocations_count_total_.fetch_sub(1, std::memory_order_relaxed);
  }
}

}  // namespace imp
