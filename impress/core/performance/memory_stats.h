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

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <thread>  // NOLINT: Need to sort by thread id.

#include "core/config.h"

#if IMP_PLATFORM(MACOS) || IMP_PLATFORM(IOS)
#define SUPPORT_CALLSTACKS 1
#else
#define SUPPORT_CALLSTACKS 0
#endif

#if IMP_PLATFORM(WASM) || IMP_PLATFORM(MACOS) || IMP_PLATFORM(ANDROID) || \
    IMP_PLATFORM(IOS)
#define SUPPORT_MEMORY_TRACKING 1
#else
#define SUPPORT_MEMORY_TRACKING 0
#endif

namespace imp {

// Singleton class for tracking memory usage within an Impress app.
// Allows querying of the total memory usage of the process and the number of
// allocations made so far.
// Also tracks the memory usage of the calling thread per-frame to be used by
// the profiler.
class MemoryStats {
 public:
// Maximum number of call stacks to store before looping.
// Memory usage increases with max callstack depth.
#if SUPPORT_CALLSTACKS
  static constexpr int kMaxCallstacks = 1024 * 1024;
#else
  static constexpr int kMaxCallstacks = 0;
#endif
  static constexpr int kMaxCallstackDepth = 16;
  struct Callstack {
    std::array<void*, kMaxCallstackDepth> callstack;
    uint32_t depth;  // 32 bits for alignment, could be a byte otherwise.
    uint32_t size;
    std::thread::id thread_id;
  };
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
#if SUPPORT_MEMORY_TRACKING
    return true;
#else
    return false;
#endif
  }
  // Returns true if memory tracking is supported on the current platform.
  static constexpr bool IsCallstackTrackingSupported() {
#if SUPPORT_CALLSTACKS
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
  Callstack& GetCallstack(size_t index) {
    if (kMaxCallstacks == 0) {
      static Callstack empty_callstack{};
      return empty_callstack;
    }
    return callstacks_[index % kMaxCallstacks];
  }
  bool IsRecordingCallstacks() {
    if (!IsCallstackTrackingSupported()) {
      return false;
    }
    return record_callstacks_.load(std::memory_order_relaxed);
  }
  void SetRecordingCallstacks(bool value) {
    if (!IsCallstackTrackingSupported()) {
      return;
    }
    return record_callstacks_.store(value, std::memory_order_relaxed);
  }
  size_t GetCallstackIndex() {
    return callstack_index_.load(std::memory_order_relaxed);
  }

 private:
  MemoryStats() = default;
  ~MemoryStats() = default;

  // Thread-safe global counter for tracked memory usage.
  std::atomic<size_t> allocated_bytes_total_{0};
  // Thread-safe global counter for tracked allocations count.
  std::atomic<size_t> allocations_count_total_{0};
  std::array<MemoryStats::Callstack, kMaxCallstacks> callstacks_;
  std::atomic<size_t> callstack_index_{0};
  std::atomic<bool> record_callstacks_{false};
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_MEMORY_STATS_H_
