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

#ifndef THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_PROFILER_STRUCTS_H_
#define THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_PROFILER_STRUCTS_H_

#include <cstdint>
#include <thread>  // NOLINT: Need to get current thread id.

#include "absl/strings/string_view.h"

namespace imp {

// Interface for results to unify code paths for main and worker thread samples.
struct ProfileResult {
  virtual ~ProfileResult() = default;
  virtual uint64_t GetStartTimeNanos() const = 0;
  virtual uint64_t GetEndTimeNanos() const = 0;
  virtual absl::string_view GetName() const = 0;
  virtual uint64_t GetSampleEndId() const = 0;
  virtual std::thread::id GetThreadId() const = 0;
};

// All the data contained in a single sample in the profiler.
struct MainThreadProfileResult final : ProfileResult {
  absl::string_view name;
  // Int16 to save memory since kMaxSamples is < 16k.
  uint16_t sample_end_id;
  // Int32 to save memory since 32 bits gives 4 full seconds per frame.
  uint32_t relative_start_time_ns;
  uint32_t relative_end_time_ns;

  inline uint64_t GetStartTimeNanos() const override {
    return static_cast<uint64_t>(relative_start_time_ns);
  }
  inline uint64_t GetEndTimeNanos() const override {
    return static_cast<uint64_t>(relative_end_time_ns);
  }
  inline absl::string_view GetName() const override { return name; }
  inline uint64_t GetSampleEndId() const override {
    return static_cast<uint64_t>(sample_end_id);
  }
  std::thread::id GetThreadId() const override;
};

// Worker thread profile result.
// These have no concept of a frame and are therefore not relative to one.
// This means the size needed to store the start/end times and id is larger.
struct WorkerProfileResult final : ProfileResult {
  absl::string_view name;
  uint64_t sample_end_id;
  uint64_t start_time_ns;
  uint64_t end_time_ns;
  std::thread::id thread_id;

  inline uint64_t GetStartTimeNanos() const override { return start_time_ns; }
  inline uint64_t GetEndTimeNanos() const override { return end_time_ns; }
  inline absl::string_view GetName() const override { return name; }
  inline uint64_t GetSampleEndId() const override { return sample_end_id; }
  inline std::thread::id GetThreadId() const override { return thread_id; }
};

struct FrameMetaData {
  uint32_t total_duration_ns;
  uint32_t sample_count;
  uint64_t frame_start_time_ns;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_PROFILER_STRUCTS_H_
