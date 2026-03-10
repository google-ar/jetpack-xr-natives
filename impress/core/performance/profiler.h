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

#ifndef THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_PROFILER_H_
#define THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_PROFILER_H_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <thread>  // NOLINT: Need to get current thread id.
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "core/config.h"
#include "core/performance/memory_stats.h"
#include "core/performance/profiler_structs.h"

namespace imp {

// Profiler is a static class compiled when IMP_DEV_RUNTIME=1 is defined.
// It keeps track of samples from IMP_TRACE containing all the data defined in
// the ProfileResult struct.
//
// The profiler is paused by default. It must be started by calling
// AdvanceFrame() to record samples.
//
// AddSample, RecordCurrentFrameSampleEndTime, and GetCachedThreadId are
// thread-safe however all other functions are designed to be called from the
// main thread only.
class Profiler {
 public:
  // Id used for samples that should not be recorded.
  static constexpr int64_t kInvalidProfileResultId = -1;
  // Name of the main thread.
  static constexpr absl::string_view kMainThreadName = "Main Thread";
  // Maximum number of frames that can be stored.
  // Increasing this value will increase the memory footprint of the profiler.
  static constexpr int kMaxFrames = 300;
  static constexpr int kMaxWorkerSamples = 64;
  // Maximum number of samples that can be stored for a single frame.
  // Increasing this value will increase the memory footprint of the profiler.
#if IMP_PLATFORM(WASM)
  // Memory is tighter in WASM. This limit may be too small for large apps.
  static constexpr int kMaxSamples = 1024;
#else
  static constexpr int kMaxSamples = 4096;
#endif
  Profiler() = delete;
  // Returns how many frames have been recorded so far.
  static int GetCurrentFrameIndex() { return frame_index_; }
  // Adds a profiling sample to the current frame.
  // Returns the index of the array the sample belongs to and its index within
  // that array.
  // Returns {-1,-1} if there was no room to add the sample or the profiler is
  // not recording.
  static int64_t AddSample(absl::string_view name);
  // Set whether the profiler is paused or recording samples.
  static void SetPaused(bool paused) { paused_ = paused; }
  // Tells the profiler a new frame has started.
  static void AdvanceFrame();
  // Returns the profile samples for the frame at the given index.
  static std::array<MainThreadProfileResult, kMaxSamples>& GetSamples(
      int frame_index);
  // Returns the number of samples for the frame at the given index.
  static int GetSampleCount(int frame_index);
  // Returns the duration of time FilamentHost::RenderNextFrame() took for the
  // given frame in nanoseconds.
  static uint32_t GetRenderNextFrameDurationNanos(int frame_index);
  // Returns the total duration of the given frame in nanoseconds.
  static uint32_t GetTotalFrameDurationNanos(int frame_index);
  // Sets the end time of the given sample index to the current time.
  static void RecordSampleEndTime(int64_t id);
  // Returns the main thread id.
  static std::thread::id GetMainThreadId() { return main_thread_id_.load(); }
  // Returns the id of the thread executing this function.
  // Value is cached using thread_local to avoid a lot of system calls.
  static std::thread::id GetCachedThreadId();
  // Is the profiler currently recording samples?
  static bool IsRecording() { return is_recording_; }
  // Returns the total memory allocated by the app so far.
  static size_t GetMemoryUsageBytes() {
    return MemoryStats::Get().GetMemoryUsageBytes();
  }
  // Returns the total number of allocations made by the app so far.
  static size_t GetAllocationsCountTotal() {
    return MemoryStats::Get().GetAllocationsCountTotal();
  }
  // Returns true if memory tracking is enabled for the current platform.
  static constexpr bool IsMemoryTrackingSupported() {
    return MemoryStats::IsMemoryTrackingSupported();
  }
  // Returns true if the memory graph is enabled for the current platform.
  static constexpr bool IsMemoryGraphSupported() {
    return MemoryStats::IsMemoryGraphSupported();
  }
  // Returns true if the data for the given frame is available.
  static bool HasFrameRecorded(int frame_index);
  // Records a name for the thread executing this function.
  // Used to identify the thread in the profiler UI.
  static void SetThreadName(absl::string_view name);
  // Returns the name of the thread with the given id.
  // Returns a string representation of the thread id if no name has been set,
  // in this case an allocation will occur the first time the name is requested.
  static absl::string_view GetThreadName(std::thread::id thread_id);
  // Returns the frame metadata for the frame at the given index.
  static FrameMetaData& GetFrameMetaData(int frame_index);
  // Returns the samples for a given thread that overlap the given time range.
  static std::vector<WorkerProfileResult> GetWorkerThreadSamples(
      uint64_t start_time, uint64_t end_time, std::thread::id thread_id);
  // Returns all worker samples that were running between the given times.
  static absl::flat_hash_map<std::thread::id, std::vector<WorkerProfileResult>>
  GetAllWorkerThreadsSamples(uint64_t start_time, uint64_t end_time);
  // Returns thread ids for all worker threads that have ever recorded samples.
  static std::vector<std::thread::id> GetWorkerThreadIds();
  // Returns true if memory call stacks are currently being recorded.
  static bool IsRecordingCallstacks() {
    if (!MemoryStats::IsCallstackTrackingSupported()) {
      return false;
    }
    return is_recording_callstacks_.load(std::memory_order_relaxed);
  }
  // Sets whether memory call stacks are being recorded.
  static void SetRecordingCallstacks(bool is_recording) {
    if (!MemoryStats::IsCallstackTrackingSupported()) {
      return;
    }
    is_recording_callstacks_.store(is_recording, std::memory_order_relaxed);
  }

 private:
  // Main thread members:

  inline static uint32_t GetCurrentFrameTimeNanos();

  // Returns the duration of time since the last call to AdvanceFrame() in ns.
  // uint32 so max it can return is 4s per frame.
  inline static std::atomic<std::thread::id> main_thread_id_;
  inline static std::atomic<bool> is_recording_callstacks_{false};
  inline static std::array<FrameMetaData, kMaxFrames> frame_metadata_;
  inline static int frame_index_ = 0;
  inline static int sample_index_ = 0;
  inline static bool paused_ = false;
  // Duplicate of is_recording_ but without thread safety for the main thread.
  // Prevents the need for an atomic load for each main thread AddSample call.
  inline static bool is_recording_main_thread_ = true;
  inline static uint16_t id_counter_ = 0;  // Int16 as kMaxSamples is < 16k.
  inline static uint16_t end_id_counter_ = 0;
  inline static int64_t last_frame_start_time_ns_ = 0;
  static void RecordMainThreadSampleEndTime(int64_t id);
  static int64_t AddMainThreadSample(absl::string_view name);

  // Thread-safe members:

  static int64_t GetCurrentTimeNanos();

  // Whether the profiler is recording samples.
  inline static std::atomic<bool> is_recording_{true};

  static void RecordWorkerThreadSampleEndTime(int64_t id);
  static int64_t AddWorkerThreadSample(absl::string_view name);
  static size_t FindSampleIndexUpperBound(uint64_t end_time);

  static bool CheckOnMainThread() {
    return main_thread_id_.load() == GetCachedThreadId();
  }
};
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_PROFILER_H_
