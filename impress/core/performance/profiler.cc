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

#include "absl/base/optimization.h"
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
#include "core/performance/profiler_state.h"
#include "core/performance/profiler_structs.h"
#if IMP_PLATFORM(WASM)
#include <emscripten/emscripten.h>
#endif

namespace imp {

namespace {

// Max recorded frames, taken from MainThreadProfilerState.
constexpr int kMaxFrames = MainThreadProfilerState::kMaxFrames;

// Max main thread samples per frame, taken from MainThreadProfilerState.
constexpr int kMaxSamples = MainThreadProfilerState::kMaxSamples;

// Max worker thread samples, taken from WorkerThreadProfilerState.
constexpr int kMaxWorkerSamples = WorkerThreadProfilerState::kMaxWorkerSamples;

// Global pointer to the profiler state for fast access in hot paths.
// This is initialized in EnsureProfilerState().
MainThreadProfilerState* g_main_state = nullptr;
WorkerThreadProfilerState* g_worker_state = nullptr;

static void EnsureProfilerState() {
  // This is a roundabout way to follow totw/110 without using NoDestructor.
  // We can't use NoDestructor because it allocates this memory at app launch.
  // Our goal with this setup is to lazy-load the memory when recording starts.
  static MainThreadProfilerState* const main_state =
      new MainThreadProfilerState();
  static WorkerThreadProfilerState* const worker_state =
      new WorkerThreadProfilerState();

  g_main_state = main_state;
  g_worker_state = worker_state;
}

}  // namespace

void Profiler::SetPaused(bool paused) {
  paused_ = paused;
  if (!paused) EnsureProfilerState();
}

int Profiler::GetCurrentFrameIndex() {
  if (!g_main_state) return 0;
  return g_main_state->frame_index;
}

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

  uint16_t current_id = g_main_state->id_counter++;

  // Would be nice to log here but it causes tests to fail.
  // The console editor widget test fails if we log here.
  if (current_id >= kMaxSamples) return kInvalidProfileResultId;

  uint32_t start_time = GetCurrentFrameTimeNanos();
  int callstack_index = MemoryStats::Get().IsRecordingCallstacks()
                            ? MemoryStats::Get().GetCallstackIndex()
                            : -1;

  MainThreadProfileResult& sample =
      g_main_state->samples[g_main_state->sample_index][current_id];
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
  if (!is_recording_.load(std::memory_order_acquire))
    return kInvalidProfileResultId;

  std::thread::id thread_id = GetCachedThreadId();

  // Cache thread registration.
  thread_local bool thread_registered = false;

  if (ABSL_PREDICT_FALSE(!thread_registered)) {
    absl::MutexLock lock(g_worker_state->thread_ids_mu);
    g_worker_state->thread_ids.push_back(thread_id);
    thread_registered = true;
  }

  size_t memory_usage_bytes =
      MemoryStats::Get().GetMemoryBytesAllocatedOnThisThread();
  size_t allocations_count =
      MemoryStats::Get().GetAllocationsCountOnThisThread();
  int callstack_index = MemoryStats::Get().IsRecordingCallstacks()
                            ? MemoryStats::Get().GetCallstackIndex()
                            : -1;

  size_t current_index;
  {
    absl::MutexLock lock(g_worker_state->worker_samples_mu);
    // Get current time inside the mutex to ensure strictly increasing start
    // times in the circular buffer.
    uint64_t start_time = GetCurrentTimeNanos();

    current_index = g_worker_state->worker_sample_index;
    g_worker_state->worker_sample_index =
        (g_worker_state->worker_sample_index + 1) % kMaxWorkerSamples;

    WorkerProfileResult& sample = g_worker_state->worker_samples[current_index];
    sample.name = name;
    sample.start_time_ns = start_time;
    sample.thread_id = thread_id;
    sample.end_time_ns = start_time;
    sample.allocation_bytes = memory_usage_bytes;
    sample.allocation_count = allocations_count;
    sample.callstack_start_index = callstack_index;
    sample.callstack_end_index = callstack_index;
  }

  return static_cast<int64_t>(current_index);
}

void Profiler::AdvanceFrame() {
  if (ABSL_PREDICT_FALSE(!g_main_state)) {
    // Prevents large spike on first recorded frame.
    last_frame_start_time_ns_ = GetCurrentTimeNanos();
    return;
  }

  static bool main_thread_set = false;
  if (ABSL_PREDICT_FALSE(!main_thread_set)) {
    SetThreadName(kMainThreadName);
    absl::MutexLock lock(g_worker_state->thread_ids_mu);
    g_worker_state->thread_ids.push_back(GetCachedThreadId());
    main_thread_set = true;
  }

  g_main_state->frame_metadata[g_main_state->sample_index].sample_count =
      g_main_state->id_counter;

  // Record frame timings even if paused.
  // If we don't there would be a big frametime spike on resuming.
  int64_t new_frame_time_ns = GetCurrentTimeNanos();
  int64_t frame_duration_ns = new_frame_time_ns - last_frame_start_time_ns_;
  last_frame_start_time_ns_ = new_frame_time_ns;

  if (paused_) {
    // Pause can happen at any time during a frame.
    // We don't want that to mess with the frame currently being recorded.
    // is_recording updates on a per-frame basis to prevent this.
    is_recording_.store(false, std::memory_order_release);
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
    g_main_state->frame_metadata[g_main_state->sample_index].total_duration_ns =
        static_cast<uint32_t>(clamped_frame_duration_ns);
  }

  MemoryStats::Get().SetRecordingCallstacks(
      is_recording_callstacks_.load(std::memory_order_relaxed));

  is_recording_.store(true, std::memory_order_release);
  MemoryStats::Get().ResetMemoryCountersForThisThread();
  is_recording_main_thread_ = true;
  g_main_state->id_counter = 0;
  g_main_state->end_id_counter = 0;
  g_main_state->frame_index++;
  int next_sample_index = (g_main_state->sample_index + 1) % kMaxFrames;
  // Zero out the sample count for the next frame.
  g_main_state->frame_metadata[next_sample_index].sample_count = 0;
  g_main_state->frame_metadata[next_sample_index].frame_start_time_ns =
      new_frame_time_ns;
  g_main_state->sample_index = next_sample_index;
  main_thread_id_.store(GetCachedThreadId());
}

absl::StatusOr<const std::array<MainThreadProfileResult, kMaxSamples>*>
Profiler::GetSamples(int frame_index) {
  if (!HasFrameRecorded(frame_index)) {
    return absl::OutOfRangeError(
        absl::StrFormat("Frame %d not found or overwritten", frame_index));
  }

  return &g_main_state->samples[frame_index % kMaxFrames];
}

absl::StatusOr<int> Profiler::GetSampleCount(int frame_index) {
  if (!HasFrameRecorded(frame_index)) {
    return absl::OutOfRangeError(
        absl::StrFormat("Frame %d not found or overwritten", frame_index));
  }

  return g_main_state->frame_metadata[frame_index % kMaxFrames].sample_count;
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
  MainThreadProfileResult& sample =
      g_main_state->samples[frame_index % kMaxFrames][0];
  return sample.relative_end_time_ns - sample.relative_start_time_ns;
}

absl::StatusOr<FrameMetaData> Profiler::GetFrameMetaData(int frame_index) {
  if (!HasFrameRecorded(frame_index)) {
    return absl::OutOfRangeError(
        absl::StrFormat("Frame %d not found or overwritten", frame_index));
  }
  return g_main_state->frame_metadata[frame_index % kMaxFrames];
}

absl::StatusOr<uint32_t> Profiler::GetTotalFrameDurationNanos(int frame_index) {
  if (!HasFrameRecorded(frame_index)) {
    return absl::OutOfRangeError(
        absl::StrFormat("Frame %d not found or overwritten", frame_index));
  }
  return g_main_state->frame_metadata[frame_index % kMaxFrames]
      .total_duration_ns;
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
  MainThreadProfileResult& sample =
      g_main_state->samples[g_main_state->sample_index][id];
  sample.relative_end_time_ns = GetCurrentFrameTimeNanos();
  sample.sample_end_id = g_main_state->end_id_counter++;
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

  absl::MutexLock lock(g_worker_state->worker_samples_mu);
  WorkerProfileResult& sample = g_worker_state->worker_samples[id];
  sample.end_time_ns = end_time;
  sample.sample_end_id = g_worker_state->worker_end_id_counter++;

  sample.allocation_bytes = memory_allocated - sample.allocation_bytes;
  sample.allocation_count = allocations_count - sample.allocation_count;
  sample.callstack_end_index = callstack_index;
}

absl::flat_hash_map<std::thread::id, std::vector<WorkerProfileResult>>
Profiler::GetAllWorkerThreadsSamples(uint64_t start_time, uint64_t end_time) {
  absl::flat_hash_map<std::thread::id, std::vector<WorkerProfileResult>>
      samples;
  // Use acquire load to synchronize with AdvanceFrame.
  if (!is_recording_.load(std::memory_order_acquire) && !g_worker_state)
    return samples;

  absl::MutexLock lock(g_worker_state->worker_samples_mu);

  size_t end_logical_idx = FindSampleIndexUpperBound(end_time);

  for (size_t i = 0; i < end_logical_idx; ++i) {
    size_t actual_idx =
        (g_worker_state->worker_sample_index + i) % kMaxWorkerSamples;
    WorkerProfileResult& sample = g_worker_state->worker_samples[actual_idx];

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
  // Use acquire load to synchronize with AdvanceFrame.
  if (!is_recording_.load(std::memory_order_acquire) && !g_worker_state)
    return samples;

  absl::MutexLock lock(g_worker_state->worker_samples_mu);

  size_t end_logical_idx = FindSampleIndexUpperBound(end_time);

  for (size_t i = 0; i < end_logical_idx; ++i) {
    size_t actual_idx =
        (g_worker_state->worker_sample_index + i) % kMaxWorkerSamples;
    WorkerProfileResult& sample = g_worker_state->worker_samples[actual_idx];

    if (sample.end_time_ns < start_time || sample.thread_id != thread_id) {
      continue;
    }

    samples.push_back(sample);
  }
  return samples;
}

size_t Profiler::FindSampleIndexUpperBound(uint64_t end_time)
    ABSL_SHARED_LOCKS_REQUIRED(g_worker_state->worker_samples_mu) {
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
    size_t actual_idx =
        (g_worker_state->worker_sample_index + mid) % kMaxWorkerSamples;
    if (g_worker_state->worker_samples[actual_idx].start_time_ns > end_time) {
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
  // Use acquire load to synchronize with AdvanceFrame.
  if (!is_recording_.load(std::memory_order_acquire) && !g_main_state)
    return false;

  return (frame_index >= 0) &&
         (frame_index > g_main_state->frame_index - kMaxFrames) &&
         (frame_index < g_main_state->frame_index);
}

void Profiler::SetThreadName(absl::string_view name) {
  absl::MutexLock lock(g_worker_state->thread_name_mu);
  g_worker_state->thread_names[GetCachedThreadId()] = name;
}

absl::string_view Profiler::GetThreadName(std::thread::id thread_id) {
  absl::MutexLock lock(g_worker_state->thread_name_mu);
  auto it = g_worker_state->thread_names.find(thread_id);
  if (it != g_worker_state->thread_names.end()) {
    return it->second;
  }

  // If the thread name is not found, generate a hash and use that as the name.
  size_t hash = absl::Hash<std::thread::id>()(thread_id);
  g_worker_state->thread_names.emplace(thread_id,
                                       absl::StrFormat("Thread %d", hash));
  return g_worker_state->thread_names[thread_id];
}

std::vector<std::thread::id> Profiler::GetThreadIds() {
  // Use acquire load to synchronize with AdvanceFrame.
  if (!is_recording_.load(std::memory_order_acquire) && !g_worker_state)
    return {};

  absl::ReaderMutexLock lock(g_worker_state->thread_ids_mu);
  return g_worker_state->thread_ids;
}

std::thread::id MainThreadProfileResult::GetThreadId() const {
  return Profiler::GetMainThreadId();
}
}  // namespace imp
