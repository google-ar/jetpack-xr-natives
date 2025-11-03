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
#include <cstdint>
#include <thread>  // NOLINT: Need to get current thread id.

#include "absl/strings/string_view.h"
#include "core/config.h"

namespace imp {

// All the data contained in a single sample in the profiler.
struct ProfileResult {
  absl::string_view name;
  // Int16 to save memory since kMaxFrames is < 16k.
  uint16_t sample_id;
  uint16_t sample_end_id;
  // Int32 to save memory since 32 bits gives 4 full seconds per frame.
  uint32_t duration_ns;
  std::thread::id thread_id;
};

// Which array and index within that array a sample belongs to.
struct SampleIndices {
  int sample_frame_index;
  int64_t sample_id;
};

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
  // Maximum number of frames that can be stored.
  // Increasing this value will increase the memory footprint of the profiler.
  static constexpr int kMaxFrames = 300;
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
  static SampleIndices AddSample(absl::string_view name);
  // Set whether the profiler is paused or recording samples.
  static void SetPaused(bool paused) { paused_ = paused; }
  // Tells the profiler a new frame has started.
  static void AdvanceFrame();
  // Returns the profile samples for the frame at the given index.
  static std::array<ProfileResult, kMaxSamples>& GetSamples(int frame_index);
  // Returns the number of samples for the frame at the given index.
  static int GetSampleCount(int frame_index);
  // Returns the duration of time FilamentHost::RenderNextFrame() took for the
  // given frame in nanoseconds.
  static uint32_t GetRenderNextFrameDurationNanos(int frame_index);
  // Returns the total duration of the given frame in nanoseconds.
  static uint32_t GetTotalFrameDurationNanos(int frame_index);
  // Sets the end time of the given sample index to the current time.
  static void RecordCurrentFrameSampleEndTime(int sample_index, int64_t id);
  // Returns the main thread id.
  static std::thread::id GetMainThreadId() { return main_thread_id_; }
  // Returns the id of the thread executing this function.
  // Value is cached using thread_local to avoid a lot of system calls.
  static std::thread::id GetCachedThreadId();
  // Is the profiler currently recording samples?
  static bool IsRecording() { return is_recording_; }
  // Returns true if the frame index has been recorded and is still available.
  static bool HasFrameRecorded(int frame_index);

 private:
  static int64_t GetCurrentTimeNanos();
  // Returns the duration of time since the last call to AdvanceFrame() in ns.
  // uint32 so max it can return is 4s per frame.
  inline static uint32_t GetCurrentFrameTimeNanos();
  static std::array<std::array<ProfileResult, kMaxSamples>, kMaxFrames>
      samples_;
  static std::array<int, kMaxFrames> sample_counts_;
  static std::atomic<int> sample_index_;
  static std::array<uint32_t, kMaxFrames> frame_durations_ns_;
  static int frame_index_;
  static std::thread::id main_thread_id_;
  static bool paused_;
  static std::atomic<uint16_t> id_counter_;  // Int16 as kMaxSamples is < 16k.
  static std::atomic<bool> is_recording_;
  static std::atomic<uint16_t> end_id_counter_;
  static std::atomic<int64_t> last_frame_start_time_ns_;
};
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_PROFILER_H_
