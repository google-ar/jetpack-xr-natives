/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_PROFILER_STATE_H_
#define THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_PROFILER_STATE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>  // NOLINT: Need to store thread ids.
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "core/config.h"
#include "core/performance/profiler_structs.h"

namespace imp {

// Struct to hold the state of the main thread profiler.
struct MainThreadProfilerState {
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

  // Current main thread frame index.
  int frame_index = 0;

  // Current index into the main thread samples circular buffer.
  int sample_index = 0;

  // Main thread sample id counter.
  uint16_t id_counter = 0;

  // Main thread sample end id counter.
  uint16_t end_id_counter = 0;

  // Metadata for each main thread frame.
  std::array<FrameMetaData, kMaxFrames> frame_metadata;

  // Main thread samples.
  std::array<std::array<MainThreadProfileResult, kMaxSamples>, kMaxFrames>
      samples;
};

// Struct to hold the state of the worker thread profiler.
struct WorkerThreadProfilerState {
  // Maximum number of worker samples that can be stored.
  // Increasing this value will increase the memory footprint of the profiler.
  static constexpr int kMaxWorkerSamples = 65536;

  // Mutex for the thread names map.
  absl::Mutex thread_name_mu;

  // Mutex for the worker samples circular buffer.
  absl::Mutex worker_samples_mu;

  // Mutex for the worker thread ids vector.
  absl::Mutex thread_ids_mu;

  // Worker thread samples.
  std::array<WorkerProfileResult, kMaxWorkerSamples> worker_samples
      ABSL_GUARDED_BY(worker_samples_mu);

  // Stores the thread name for each thread id.
  absl::flat_hash_map<std::thread::id, std::string> thread_names
      ABSL_GUARDED_BY(thread_name_mu);

  // Thread ids for all threads that have ever recorded samples.
  std::vector<std::thread::id> thread_ids ABSL_GUARDED_BY(thread_ids_mu);

  // Worker thread sample end id counter.
  uint64_t worker_end_id_counter ABSL_GUARDED_BY(worker_samples_mu) = 0;

  // Current index into the worker thread samples circular buffer.
  size_t worker_sample_index ABSL_GUARDED_BY(worker_samples_mu) = 0;
};
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_PROFILER_STATE_H_
