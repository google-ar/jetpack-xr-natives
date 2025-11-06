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
#include <cstdint>
#include <thread>  // NOLINT: Need to get current thread id.

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"

namespace imp {

// Static member definitions
bool Profiler::paused_ = false;
std::atomic<bool> Profiler::is_recording_{true};
int Profiler::frame_index_ = 0;
std::atomic<int> Profiler::sample_index_{0};
std::atomic<uint16_t> Profiler::id_counter_{0};
std::thread::id Profiler::main_thread_id_;
std::array<std::array<ProfileResult, Profiler::kMaxSamples>,
           Profiler::kMaxFrames>
    Profiler::samples_;
std::array<int, Profiler::kMaxFrames> Profiler::sample_counts_;

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

  int64_t start_time = absl::GetCurrentTimeNanos();
  // Not atomic since this index is only written on the main thread.
  // We do need to store it locally though just in case the value changes
  // between the array access and the return statement.
  int current_frame_index = sample_index_.load(std::memory_order_relaxed);
  samples_[current_frame_index][current_id] = {name, current_id, start_time,
                                               start_time, GetCachedThreadId()};
  return {current_frame_index, current_id};
}

void Profiler::AdvanceFrame() {
  int sample_index = sample_index_.load(std::memory_order_relaxed);
  sample_counts_[sample_index] = id_counter_.load(std::memory_order_relaxed);
  if (paused_) {
    // Pause can happen at any time during a frame.
    // We don't want that to mess with the frame currently being recorded.
    // is_recording updates on a per-frame basis to prevent this.
    if (is_recording_.load(std::memory_order_relaxed)) {
      is_recording_.store(false, std::memory_order_relaxed);
    }
    return;
  }

  is_recording_.store(true, std::memory_order_relaxed);
  id_counter_.store(0, std::memory_order_relaxed);
  frame_index_++;
  int next_sample_index = (sample_index + 1) % kMaxFrames;
  // Zero out the sample count for the next frame.
  sample_counts_[next_sample_index] = 0;
  sample_index_.store(next_sample_index, std::memory_order_relaxed);
  main_thread_id_ = GetCachedThreadId();
}

std::array<ProfileResult, Profiler::kMaxSamples>&
Profiler::GetCurrentFrameSamples() {
  return samples_[sample_index_.load(std::memory_order_relaxed)];
}

std::array<ProfileResult, Profiler::kMaxSamples>& Profiler::GetSamples(
    int frame_index) {
  return samples_[frame_index % kMaxFrames];
}

int Profiler::GetSampleCount(int frame_index) {
  return sample_counts_[frame_index % kMaxFrames];
}

float Profiler::GetFrameDuration(int frame_index) {
  // We assume that the first PROFILE call encapsulates all other samples.
  // If not this will require changes and influence the performance impact.
  // We would need to iterate over the entire sample vector to find the earliest
  // start time and latest end time in that case.
  // FilamentHost::RenderNextFrame() is the default "root" sample.
  ProfileResult sample = samples_[frame_index % kMaxFrames][0];
  return sample.end_time - sample.start_time;
}

void Profiler::RecordCurrentFrameSampleEndTime(int sample_index, int64_t id) {
  if (id < 0 || id >= kMaxSamples) {
    IMP_LOG(imp::WARNING)
        << "Invalid index used for Profiler::RecordCurrentFrameSampleEndTime: "
        << id;
    return;
  }
  samples_[sample_index][id].end_time = absl::GetCurrentTimeNanos();
}

std::thread::id Profiler::GetCachedThreadId() {
  thread_local const std::thread::id cached_id = std::this_thread::get_id();
  return cached_id;
}
}  // namespace imp
