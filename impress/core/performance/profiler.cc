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

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <thread>  // NOLINT: Need to get current thread id.
#include <vector>

#include "absl/base/attributes.h"
#include "absl/base/const_init.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "core/config.h"
#include "core/performance/memory_stats.h"
#include "core/performance/profiler_structs.h"
#if IMP_PLATFORM(WASM)
#include <emscripten/emscripten.h>
#endif

namespace imp {

namespace {
// Main thread members:
std::array<std::array<MainThreadProfileResult, Profiler::kMaxSamples>,
           Profiler::kMaxFrames>
    samples;

// Thread-safe members:
ABSL_CONST_INIT absl::Mutex thread_name_mu(absl::kConstInit);
ABSL_CONST_INIT absl::Mutex worker_samples_mu(absl::kConstInit);
ABSL_CONST_INIT absl::Mutex worker_thread_ids_mu(absl::kConstInit);
absl::flat_hash_map<std::thread::id, std::string> thread_names
    ABSL_GUARDED_BY(thread_name_mu);
std::array<WorkerProfileResult, Profiler::kMaxWorkerSamples> worker_samples
    ABSL_GUARDED_BY(worker_samples_mu);
uint64_t worker_end_id_counter ABSL_GUARDED_BY(worker_samples_mu) = 0;
size_t worker_sample_index ABSL_GUARDED_BY(worker_samples_mu) = 0;
std::vector<std::thread::id> worker_thread_ids
    ABSL_GUARDED_BY(worker_thread_ids_mu);
}  // namespace

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
  return static_cast<uint32_t>(GetCurrentTimeNanos() -
                               last_frame_start_time_ns_);
}

int64_t Profiler::AddSample(const absl::string_view name) {
  if (CheckOnMainThread()) {
    return AddMainThreadSample(name);
  } else {
    return AddWorkerThreadSample(name);
  }
}

int64_t Profiler::AddMainThreadSample(const absl::string_view name) {
  if (!is_recording_main_thread_) return kInvalidProfileResultId;

  uint16_t current_id = id_counter_++;

  // Would be nice to log here but it causes tests to fail.
  // The console editor widget test fails if we log here.
  if (current_id >= kMaxSamples) return kInvalidProfileResultId;

  uint32_t start_time = GetCurrentFrameTimeNanos();
  int callstack_index = MemoryStats::Get().IsRecordingCallstacks()
                            ? MemoryStats::Get().GetCallstackIndex()
                            : -1;

  MainThreadProfileResult& sample = samples[sample_index_][current_id];
  sample.name = name;
  sample.relative_start_time_ns = start_time;
  sample.relative_end_time_ns = start_time;
  sample.sample_end_id = current_id;
  sample.allocation_bytes =
      MemoryStats::Get().GetMemoryBytesAllocatedOnThisThread();
  sample.allocation_count =
      MemoryStats::Get().GetAllocationsCountOnThisThread();
  sample.callstack_start_index = callstack_index;
  sample.callstack_end_index = callstack_index;

  return static_cast<int64_t>(current_id);
}

int64_t Profiler::AddWorkerThreadSample(const absl::string_view name) {
  if (!is_recording_.load(std::memory_order_relaxed))
    return kInvalidProfileResultId;

  // Get time and thread id before mutex to reduce time spent locked.
  uint64_t start_time = GetCurrentTimeNanos();
  std::thread::id thread_id = GetCachedThreadId();
  size_t memory_usage_bytes =
      MemoryStats::Get().GetMemoryBytesAllocatedOnThisThread();
  size_t allocations_count =
      MemoryStats::Get().GetAllocationsCountOnThisThread();
  int callstack_index = MemoryStats::Get().IsRecordingCallstacks()
                            ? MemoryStats::Get().GetCallstackIndex()
                            : -1;

  size_t current_index;
  {
    absl::MutexLock lock(worker_samples_mu);
    current_index = worker_sample_index;
    worker_sample_index = (worker_sample_index + 1) % kMaxWorkerSamples;

    WorkerProfileResult& sample = worker_samples[current_index];
    sample.name = name;
    sample.start_time_ns = start_time;
    sample.thread_id = thread_id;
    sample.end_time_ns = start_time;
    sample.allocation_bytes = memory_usage_bytes;
    sample.allocation_count = allocations_count;
    sample.callstack_start_index = callstack_index;
    sample.callstack_end_index = callstack_index;
  }

  {
    bool found = false;
    absl::MutexLock lock(worker_thread_ids_mu);
    for (int i = 0; i < worker_thread_ids.size(); ++i) {
      if (worker_thread_ids[i] == thread_id) {
        found = true;
        break;
      }
    }
    if (!found) {
      worker_thread_ids.push_back(thread_id);
    }
  }
  return static_cast<int64_t>(current_index);
}

void Profiler::AdvanceFrame() {
  static bool main_thread_set = false;
  if (!main_thread_set) {
    SetThreadName(kMainThreadName);
    main_thread_set = true;
  }

  frame_metadata_[sample_index_].sample_count = id_counter_;

  // Record frame timings even if paused.
  // If we don't there would be a big frametime spike on resuming.
  int64_t new_frame_time_ns = GetCurrentTimeNanos();
  int64_t frame_duration_ns = new_frame_time_ns - last_frame_start_time_ns_;
  last_frame_start_time_ns_ = new_frame_time_ns;

  if (paused_) {
    // Pause can happen at any time during a frame.
    // We don't want that to mess with the frame currently being recorded.
    // is_recording updates on a per-frame basis to prevent this.
    is_recording_.store(false, std::memory_order_relaxed);
    is_recording_main_thread_ = false;

    // Don't waste time recording call stacks if we're not gathering samples.
    MemoryStats::Get().SetRecordingCallstacks(false);
    return;
  }

  if (new_frame_time_ns > 0) {
    // Downcast is fine here unless a frame takes longer than 4 seconds.
    // Clamped to uint32_t max to avoid overflow in that case it happens.
    int64_t clamped_frame_duration_ns =
        std::min(frame_duration_ns,
                 static_cast<int64_t>(std::numeric_limits<uint32_t>::max()));
    frame_metadata_[sample_index_].total_duration_ns =
        static_cast<uint32_t>(clamped_frame_duration_ns);
  }

  MemoryStats::Get().SetRecordingCallstacks(
      is_recording_callstacks_.load(std::memory_order_relaxed));

  is_recording_.store(true, std::memory_order_relaxed);
  MemoryStats::Get().ResetMemoryCountersForThisThread();
  is_recording_main_thread_ = true;
  id_counter_ = 0;
  end_id_counter_ = 0;
  frame_index_++;
  int next_sample_index = (sample_index_ + 1) % kMaxFrames;
  // Zero out the sample count for the next frame.
  frame_metadata_[next_sample_index].sample_count = 0;
  frame_metadata_[next_sample_index].frame_start_time_ns = new_frame_time_ns;
  sample_index_ = next_sample_index;
  main_thread_id_.store(GetCachedThreadId());
}

absl::StatusOr<
    const std::array<MainThreadProfileResult, Profiler::kMaxSamples>*>
Profiler::GetSamples(int frame_index) {
  if (!HasFrameRecorded(frame_index)) {
    return absl::OutOfRangeError(
        absl::StrFormat("Frame %d not found or overwritten", frame_index));
  }

  return &samples[frame_index % kMaxFrames];
}

absl::StatusOr<int> Profiler::GetSampleCount(int frame_index) {
  if (!HasFrameRecorded(frame_index)) {
    return absl::OutOfRangeError(
        absl::StrFormat("Frame %d not found or overwritten", frame_index));
  }

  return frame_metadata_[frame_index % kMaxFrames].sample_count;
}

absl::StatusOr<uint32_t> Profiler::GetRenderNextFrameDurationNanos(
    int frame_index) {
  if (!HasFrameRecorded(frame_index)) {
    return absl::OutOfRangeError(
        absl::StrFormat("Frame %d not found or overwritten", frame_index));
  }

  // We assume that the first PROFILE call encapsulates all other samples.
  // If not this will require changes and influence the performance impact.
  // We would need to iterate over the entire sample vector to find the earliest
  // start time and latest end time in that case.
  // FilamentHost::RenderNextFrame() is the default "root" sample.
  MainThreadProfileResult& sample = samples[frame_index % kMaxFrames][0];
  return sample.relative_end_time_ns - sample.relative_start_time_ns;
}

absl::StatusOr<FrameMetaData> Profiler::GetFrameMetaData(int frame_index) {
  if (!HasFrameRecorded(frame_index)) {
    return absl::OutOfRangeError(
        absl::StrFormat("Frame %d not found or overwritten", frame_index));
  }
  return frame_metadata_[frame_index % kMaxFrames];
}

absl::StatusOr<uint32_t> Profiler::GetTotalFrameDurationNanos(int frame_index) {
  if (!HasFrameRecorded(frame_index)) {
    return absl::OutOfRangeError(
        absl::StrFormat("Frame %d not found or overwritten", frame_index));
  }

  return frame_metadata_[frame_index % kMaxFrames].total_duration_ns;
}

void Profiler::RecordSampleEndTime(int64_t id) {
  if (CheckOnMainThread()) {
    RecordMainThreadSampleEndTime(id);
  } else {
    RecordWorkerThreadSampleEndTime(id);
  }
}

void Profiler::RecordMainThreadSampleEndTime(int64_t id) {
  if (id < 0 || id >= kMaxSamples) {
    IMP_LOG(imp::WARNING)
        << "Invalid index used for Profiler::RecordCurrentFrameSampleEndTime: "
        << id;
    return;
  }
  MainThreadProfileResult& sample = samples[sample_index_][id];
  sample.relative_end_time_ns = GetCurrentFrameTimeNanos();
  sample.sample_end_id = end_id_counter_++;
  size_t memory_allocated_diff =
      MemoryStats::Get().GetMemoryBytesAllocatedOnThisThread() -
      sample.allocation_bytes;
  size_t allocations_count_diff =
      MemoryStats::Get().GetAllocationsCountOnThisThread() -
      sample.allocation_count;

  sample.allocation_bytes = memory_allocated_diff;
  sample.allocation_count = allocations_count_diff;

  sample.callstack_end_index = MemoryStats::Get().GetCallstackIndex();
}

void Profiler::RecordWorkerThreadSampleEndTime(int64_t id) {
  if (id < 0 || id >= kMaxWorkerSamples) {
    IMP_LOG(imp::WARNING)
        << "Invalid index used for Profiler::RecordWorkerThreadSampleEndTime: "
        << id;
    return;
  }
  // Get the end time before mutex to reduce time spent locked.
  uint64_t end_time = GetCurrentTimeNanos();
  size_t memory_allocated =
      MemoryStats::Get().GetMemoryBytesAllocatedOnThisThread();
  size_t allocations_count =
      MemoryStats::Get().GetAllocationsCountOnThisThread();
  int callstack_index = MemoryStats::Get().GetCallstackIndex();

  absl::MutexLock lock(worker_samples_mu);
  WorkerProfileResult& sample = worker_samples[id];
  sample.end_time_ns = end_time;
  sample.sample_end_id = worker_end_id_counter++;

  sample.allocation_bytes = memory_allocated - sample.allocation_bytes;
  sample.allocation_count = allocations_count - sample.allocation_count;

  sample.callstack_end_index = callstack_index;
}

absl::flat_hash_map<std::thread::id, std::vector<WorkerProfileResult>>
Profiler::GetAllWorkerThreadsSamples(uint64_t start_time, uint64_t end_time) {
  absl::flat_hash_map<std::thread::id, std::vector<WorkerProfileResult>>
      samples;
  absl::MutexLock lock(worker_samples_mu);

  size_t end_logical_idx = FindSampleIndexUpperBound(end_time);

  for (size_t i = 0; i < end_logical_idx; ++i) {
    size_t actual_idx = (worker_sample_index + i) % kMaxWorkerSamples;
    WorkerProfileResult& sample = worker_samples[actual_idx];

    // We already filtered out samples that start after end_time so we just need
    // to filter out samples that end before the start time.
    if (sample.end_time_ns < start_time) {
      continue;
    }

    samples[sample.thread_id].push_back(sample);
  }

  return samples;
}

std::vector<WorkerProfileResult> Profiler::GetWorkerThreadSamples(
    uint64_t start_time, uint64_t end_time, std::thread::id thread_id) {
  std::vector<WorkerProfileResult> samples;
  absl::MutexLock lock(worker_samples_mu);

  size_t end_logical_idx = FindSampleIndexUpperBound(end_time);

  for (size_t i = 0; i < end_logical_idx; ++i) {
    size_t actual_idx = (worker_sample_index + i) % kMaxWorkerSamples;
    WorkerProfileResult& sample = worker_samples[actual_idx];

    if (sample.end_time_ns < start_time || sample.thread_id != thread_id) {
      continue;
    }

    samples.push_back(sample);
  }
  return samples;
}

size_t Profiler::FindSampleIndexUpperBound(uint64_t end_time)
    ABSL_SHARED_LOCKS_REQUIRED(worker_samples_mu) {
  size_t low = 0, high = kMaxWorkerSamples;
  size_t end_logical_idx = kMaxWorkerSamples;

  // Worker samples are sorted by start time allowing for a small optimization.
  // We can find the first sample whose start time exceeds the end time of our
  // search to narrow down how many samples we need to check.
  // We can't narrow down the start index because we would need the samples to
  // be sorted by end time.
  // This upper bound is exclusive, we don't want the sample at that index.

  // Binary search to find the first sample starting AFTER end_time.
  while (low < high) {
    size_t mid = low + (high - low) / 2;
    size_t actual_idx = (worker_sample_index + mid) % kMaxWorkerSamples;

    if (worker_samples[actual_idx].start_time_ns > end_time) {
      end_logical_idx = mid;
      high = mid;
    } else {
      low = mid + 1;
    }
  }

  return end_logical_idx;
}

std::thread::id Profiler::GetCachedThreadId() {
  thread_local const std::thread::id cached_id = std::this_thread::get_id();
  return cached_id;
}

bool Profiler::HasFrameRecorded(int frame_index) {
  return (frame_index >= 0) && (frame_index > frame_index_ - kMaxFrames) &&
         (frame_index < frame_index_);
}

void Profiler::SetThreadName(absl::string_view name) {
  absl::MutexLock lock(thread_name_mu);
  thread_names[GetCachedThreadId()] = name;
}

absl::string_view Profiler::GetThreadName(std::thread::id thread_id) {
  absl::MutexLock lock(thread_name_mu);
  auto it = thread_names.find(thread_id);
  if (it != thread_names.end()) {
    return it->second;
  }

  // If the thread name is not found, generate a hash and use that as the name.
  size_t hash = absl::Hash<std::thread::id>()(thread_id);
  thread_names.emplace(thread_id, absl::StrFormat("Thread %d", hash));
  return thread_names[thread_id];
}

std::vector<std::thread::id> Profiler::GetWorkerThreadIds() {
  absl::MutexLock lock(worker_thread_ids_mu);
  return worker_thread_ids;
}

std::thread::id MainThreadProfileResult::GetThreadId() const {
  return Profiler::GetMainThreadId();
}
}  // namespace imp
