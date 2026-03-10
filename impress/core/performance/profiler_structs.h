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
  virtual uint32_t GetMemoryAllocated() const = 0;
  virtual uint16_t GetAllocationsCount() const = 0;
  virtual int GetCallstackStartIndex() const = 0;
  virtual int GetCallstackEndIndex() const = 0;
};

// All the data contained in a single sample in the profiler.
// Small types are used to save memory where possible since there can be
// thousands of samples per frame.
struct MainThreadProfileResult final : ProfileResult {
  absl::string_view name;

  // Int16 to save memory as this is per-frame and kMaxSamples is < 65k/frame.
  uint16_t sample_end_id;

  // Int32 to save memory since main thread samples store their start/end times
  // relative to the start of the frame they take place in. 32 bits gives up to
  // 4s of time per sample which should be more than enough.
  uint32_t relative_start_time_ns;
  uint32_t relative_end_time_ns;

  // Int32 to save memory since 32 bits gives 4GB per sample.
  uint32_t allocation_bytes;
  // Int16 to save memory since 16 bits gives 65k allocations per sample.
  uint16_t allocation_count;

  // Index of first allocation's call stack.
  int callstack_start_index;
  // Index of last allocation's call stack.
  int callstack_end_index;

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
  inline uint32_t GetMemoryAllocated() const override {
    return allocation_bytes;
  }
  inline uint16_t GetAllocationsCount() const override {
    return allocation_count;
  }
  inline int GetCallstackStartIndex() const override {
    return callstack_start_index;
  }
  inline int GetCallstackEndIndex() const override {
    return callstack_end_index;
  }
  std::thread::id GetThreadId() const override;
};

// Worker thread profile result.
// These have no concept of a frame and are therefore not relative to one.
// This means the size needed to store the start/end times and id is larger.
struct WorkerProfileResult final : ProfileResult {
  absl::string_view name;

  // Need 64-bits for worker sample ids since they do not have the concept of
  // a frame and simply increment for each sample.
  uint64_t sample_end_id;

  // Need 64 bits since start/end times are based on absolute time rather than
  // time relative to frame start.
  uint64_t start_time_ns;
  uint64_t end_time_ns;

  // Need to store the id of the thread it ran on so we can organize samples
  // by thread later.
  std::thread::id thread_id;

  // Int32 to save memory since 32 bits gives 4GB per sample.
  uint32_t allocation_bytes;
  // Int16 to save memory since 16 bits gives 65k allocations per sample.
  uint16_t allocation_count;

  // Index of first allocation's call stack.
  int callstack_start_index;
  // Index of last allocation's call stack.
  int callstack_end_index;

  inline uint64_t GetStartTimeNanos() const override { return start_time_ns; }
  inline uint64_t GetEndTimeNanos() const override { return end_time_ns; }
  inline absl::string_view GetName() const override { return name; }
  inline uint64_t GetSampleEndId() const override { return sample_end_id; }
  inline uint32_t GetMemoryAllocated() const override {
    return allocation_bytes;
  }
  inline uint16_t GetAllocationsCount() const override {
    return allocation_count;
  }
  inline int GetCallstackStartIndex() const override {
    return callstack_start_index;
  }
  inline int GetCallstackEndIndex() const override {
    return callstack_end_index;
  }
  inline std::thread::id GetThreadId() const override { return thread_id; }
};

struct FrameMetaData {
  uint32_t total_duration_ns;
  uint32_t sample_count;
  uint64_t frame_start_time_ns;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PERFORMANCE_PROFILER_STRUCTS_H_
