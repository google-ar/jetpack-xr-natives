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

#include "core/performance/profiler.h"

#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>  // NOLINT: Need to get current thread id.

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "core/common/log.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "core/config.h"
#if IMP_PLATFORM(WASM)
#include <emscripten/emscripten.h>
#endif

namespace imp {

// Static member definitions
bool Profiler::paused_ = false;
std::atomic<bool> Profiler::is_recording_{true};
int Profiler::frame_index_ = 0;
std::atomic<int> Profiler::sample_index_{0};
std::atomic<uint16_t> Profiler::id_counter_{0};
std::atomic<uint16_t> Profiler::end_id_counter_{0};
std::thread::id Profiler::main_thread_id_;
std::array<std::array<ProfileResult, Profiler::kMaxSamples>,
           Profiler::kMaxFrames>
    Profiler::samples_;
std::array<int, Profiler::kMaxFrames> Profiler::sample_counts_;
std::array<uint32_t, Profiler::kMaxFrames> Profiler::frame_durations_ns_;
std::atomic<int64_t> Profiler::last_frame_start_time_ns_{0};
absl::Mutex Profiler::mu_;
absl::flat_hash_map<std::thread::id, std::string> Profiler::thread_names_;

int64_t Profiler::GetCurrentTimeNanos() {
  // Absl's GetCurrentTimeNanos() is millisecond-accurate in wasm builds.
  // For microsecond-accurate timing, we need to use emscripten_get_now().
  // If your app does not have cross_origin_headers = True in the BUILD file,
  // this will only be 0.1ms accurate rather than 0.005ms accurate.
#if IMP_PLATFORM(WASM)
  return static_cast<int64_t>(emscripten_get_now() * 1000000.0);
#else
  return absl::GetCurrentTimeNanos();
#endif
}

uint32_t Profiler::GetCurrentFrameTimeNanos() {
  return static_cast<uint32_t>(
      GetCurrentTimeNanos() -
      last_frame_start_time_ns_.load(std::memory_order_relaxed));
}

SampleIndices Profiler::AddSample(const absl::string_view name) {
  if (!is_recording_) {
    return {-1, -1};
  }

  // AddSample may be called from any thread, we need to use atomic to
  // ensure the index we're writing to is unique.
  // This operation is safe because the array is pre-allocated, the ordering of
  // samples between threads is not important, and we're ensuring the same
  // index is never written to by multiple threads simultaneously.
  // We could also lock the entire function, but that introduces deadlock risk
  // and is likely to be slower for most hardware configurations.
  uint16_t current_id = id_counter_.fetch_add(1, std::memory_order_relaxed);

  if (current_id >= kMaxSamples) {
    // IMP_LOG(imp::WARNING) << "Attempted to add profile sample " << name
    //              << " when the sample buffer is full. Max samples: "
    //              << kMaxSamples;
    return {-1, -1};
  }

  uint32_t start_time = GetCurrentFrameTimeNanos();
  int current_frame_index = sample_index_.load(std::memory_order_relaxed);
  samples_[current_frame_index][current_id] = {name, current_id, current_id,
                                               start_time, GetCachedThreadId()};
  return {current_frame_index, current_id};
}

void Profiler::AdvanceFrame() {
  static bool main_thread_set = false;
  if (!main_thread_set) {
    SetThreadName(kMainThreadName);
    main_thread_set = true;
  }

  int sample_index = sample_index_.load(std::memory_order_relaxed);
  sample_counts_[sample_index] = id_counter_.load(std::memory_order_relaxed);

  // Record frame timings even if paused.
  // If we don't there would be a big frametime spike on resuming.
  int64_t new_frame_time_ns = GetCurrentTimeNanos();
  int64_t frame_duration_ns =
      new_frame_time_ns -
      last_frame_start_time_ns_.load(std::memory_order_relaxed);
  last_frame_start_time_ns_.store(new_frame_time_ns, std::memory_order_relaxed);

  if (paused_) {
    // Pause can happen at any time during a frame.
    // We don't want that to mess with the frame currently being recorded.
    // is_recording updates on a per-frame basis to prevent this.
    if (is_recording_.load(std::memory_order_relaxed)) {
      is_recording_.store(false, std::memory_order_relaxed);
    }
    return;
  }

  if (new_frame_time_ns > 0) {
    // Downcast is fine here unless a frame takes longer than 4 seconds.
    frame_durations_ns_[sample_index] =
        static_cast<uint32_t>(frame_duration_ns);
  }
  is_recording_.store(true, std::memory_order_relaxed);
  id_counter_.store(0, std::memory_order_relaxed);
  end_id_counter_.store(0, std::memory_order_relaxed);
  frame_index_++;
  int next_sample_index = (sample_index + 1) % kMaxFrames;
  // Zero out the sample count for the next frame.
  sample_counts_[next_sample_index] = 0;
  sample_index_.store(next_sample_index, std::memory_order_relaxed);
  main_thread_id_ = GetCachedThreadId();
}

std::array<ProfileResult, Profiler::kMaxSamples>& Profiler::GetSamples(
    int frame_index) {
  if (!HasFrameRecorded(frame_index)) return samples_[0];

  return samples_[frame_index % kMaxFrames];
}

int Profiler::GetSampleCount(int frame_index) {
  if (!HasFrameRecorded(frame_index)) return 0;

  return sample_counts_[frame_index % kMaxFrames];
}

uint32_t Profiler::GetRenderNextFrameDurationNanos(int frame_index) {
  if (!HasFrameRecorded(frame_index)) return 0;

  // We assume that the first PROFILE call encapsulates all other samples.
  // If not this will require changes and influence the performance impact.
  // We would need to iterate over the entire sample vector to find the earliest
  // start time and latest end time in that case.
  // FilamentHost::RenderNextFrame() is the default "root" sample.
  ProfileResult sample = samples_[frame_index % kMaxFrames][0];
  return sample.duration_ns;
}

uint32_t Profiler::GetTotalFrameDurationNanos(int frame_index) {
  if (!HasFrameRecorded(frame_index)) return 0;

  return frame_durations_ns_[frame_index % kMaxFrames];
}

void Profiler::RecordCurrentFrameSampleEndTime(int sample_index, int64_t id) {
  if (id < 0 || id >= kMaxSamples) {
    IMP_LOG(imp::WARNING)
        << "Invalid index used for Profiler::RecordCurrentFrameSampleEndTime: "
        << id;
    return;
  }
  ProfileResult& sample = samples_[sample_index][id];
  uint32_t start_time = sample.duration_ns;
  sample.duration_ns = GetCurrentFrameTimeNanos() - start_time;
  sample.sample_end_id =
      end_id_counter_.fetch_add(1, std::memory_order_relaxed);
}

std::thread::id Profiler::GetCachedThreadId() {
  thread_local const std::thread::id cached_id = std::this_thread::get_id();
  return cached_id;
}

bool Profiler::HasFrameRecorded(int frame_index) {
  return frame_index >= 0 && frame_index > frame_index_ - kMaxFrames &&
         frame_index < frame_index_;
}

void Profiler::SetThreadName(absl::string_view name) {
  // Moohan repo only has deprecated MutexLock constructor.
  // Didn't add nolint in case that changes.
  absl::MutexLock lock(&mu_);
  thread_names_[GetCachedThreadId()] = name;
}

absl::string_view Profiler::GetThreadName(std::thread::id thread_id) {
  // Moohan repo only has deprecated MutexLock constructor.
  // Didn't add nolint in case that changes.
  absl::MutexLock lock(&mu_);
  auto it = thread_names_.find(thread_id);
  if (it != thread_names_.end()) {
    return it->second;
  }

  // If the thread name is not found, generate a hash and use that as the name.
  size_t hash = absl::Hash<std::thread::id>()(thread_id);
  thread_names_.emplace(thread_id, absl::StrFormat("Thread %d", hash));
  return thread_names_[thread_id];
}
}  // namespace imp
