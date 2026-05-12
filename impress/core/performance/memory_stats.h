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

#ifndef THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_MEMORY_STATS_H_
#define THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_MEMORY_STATS_H_

#include <atomic>
#include <cstddef>

#include "core/config.h"

namespace imp {

// Singleton class for tracking memory usage within an Impress app.
// Allows querying of the total memory usage of the process and the number of
// allocations made so far.
// Also tracks the memory usage of the calling thread per-frame to be used by
// the profiler.
class MemoryStats {
 public:
  static MemoryStats& Get() {
    static MemoryStats instance;
    return instance;
  }

  MemoryStats(const MemoryStats&) = delete;
  MemoryStats& operator=(const MemoryStats&) = delete;

  // Increments the memory counters for the current thread.
  void IncrementMemoryCounters(size_t size);
  // Decrements the memory counters for the current thread.
  void DecrementMemoryCounters(size_t size);
  // Returns the total memory usage of the process in bytes.
  size_t GetMemoryUsageBytes();
  // Returns the total number of allocations made by the app so far.
  size_t GetAllocationsCountTotal();
  // Returns the memory allocated by this thread.
  size_t GetMemoryBytesAllocatedOnThisThread();
  // Returns the total number of allocations made by the calling thread.
  size_t GetAllocationsCountOnThisThread();
  // Resets per-frame counters.
  void ResetMemoryCountersForThisThread();
  // Returns true if memory tracking is supported on the current platform.
  static constexpr bool IsMemoryTrackingSupported() {
#if IMP_PLATFORM(WASM) || IMP_PLATFORM(MACOS) || IMP_PLATFORM(ANDROID) || \
    IMP_PLATFORM(IOS)
    return true;
#else
    return false;
#endif
  }
  // Returns true if the memory graph is enabled for the current platform.
  // This is a separate feature from allocation tracking for profiler samples.
  // Since tracking the full lifetime of allocations has significant overhead
  // for MacOS/iOS, this feature is disabled by default on those platforms.
  // However, it can be enabled with --define=PROFILER_MEMORY_GRAPH=1.
  static constexpr bool IsMemoryGraphSupported() {
#if (IMP_PLATFORM(MACOS) || IMP_PLATFORM(IOS))
#if defined(IMP_PROFILER_MEMORY_GRAPH) && IMP_PROFILER_MEMORY_GRAPH == 1
    return true;
#else
    return false;
#endif
#else
    return IsMemoryTrackingSupported();
#endif
  }

 private:
  MemoryStats() = default;
  ~MemoryStats() = default;

  // Thread-safe global counter for tracked memory usage.
  std::atomic<size_t> allocated_bytes_total_{0};
  // Thread-safe global counter for tracked allocations count.
  std::atomic<size_t> allocations_count_total_{0};
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_MEMORY_STATS_H_
