/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_SAMPLE_PROCESSOR_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_SAMPLE_PROCESSOR_HELPERS_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/strings/string_view.h"
#include "core/performance/profiler.h"
#include "core/performance/profiler_structs.h"

namespace imp::editor {

// Processed sample node, acts as a grouping of samples that have the same name
// and parent sample.
// There will be one of these for every sample in the profiler so we use
// small types to save memory where possible.
struct SampleNode {
  // Other sample groups that took place within this sample's duration.
  SampleNode* first_child;
  SampleNode* next_sibling;

  // The original profile result that this node is based on.
  const ProfileResult* result;

  // uint64 since worker samples might exceed 4s.
  // Wasteful for main thread samples but significantly simplifies code.
  uint64_t total_time_ns;

  // Number of times a ProfileResult with the same name and parent was found.
  // 16 bits to save memory as it is incredibly unlikely there will be >65k
  // profiler samples with the same name in a single frame.
  uint16_t calls;

  // Int32 to save memory since 32 bits gives 4GB per sample.
  uint32_t total_memory_allocated;
  // Int16 to save memory since 16 bits gives 65k allocations per sample.
  uint16_t total_memory_allocations_count;

  // Sets the initial values for a new sample node, clears existing children.
  void SetInitialValues(const ProfileResult* profile_result) {
    result = profile_result;
    total_time_ns =
        profile_result->GetEndTimeNanos() - profile_result->GetStartTimeNanos();
    calls = 1;
    total_memory_allocated = profile_result->GetMemoryAllocated();
    total_memory_allocations_count = profile_result->GetAllocationsCount();
    first_child = nullptr;
    next_sibling = nullptr;
  }

  // Adds a child node to this node's list of children.
  void AddChild(SampleNode* child) {
    child->next_sibling = first_child;
    first_child = child;
  }
};

struct ProcessedSamples {
  // A map relating a sample name to all samples with that name.
  absl::flat_hash_map<absl::string_view, std::vector<SampleNode*>,
                      absl::Hash<absl::string_view>>
      samples_by_name;

  // The root samples of the tree that all samples fall under.
  std::vector<SampleNode*> sample_roots;

  // Maximum depth of the tree.
  // Useful metadata for UI layouts.
  size_t max_depth = 0;

  inline void AddSample(SampleNode* sample) {
    samples_by_name[sample->result->GetName()].push_back(sample);
  }
  void AddRootSample(SampleNode* sample) {
    AddSample(sample);
    sample_roots.push_back(sample);
  }
};

class NodePool {
 public:
  virtual ~NodePool() = default;
  virtual SampleNode* Get(int index) = 0;
  virtual SampleNode* GetNext() = 0;
  inline void ResetIndex() { index_ = 0; }

 protected:
  size_t index_ = 0;
};

class WorkerNodePool final : public NodePool {
 public:
  inline SampleNode* Get(int index) override {
    return &worker_sample_node_pool_[index];
  }

  inline SampleNode* GetNext() override {
    return &worker_sample_node_pool_[index_++];
  }

 private:
  std::array<SampleNode, Profiler::kMaxWorkerSamples> worker_sample_node_pool_;
};

class MainThreadNodePool final : public NodePool {
 public:
  inline SampleNode* Get(int index) override {
    return &sample_node_pool_[index];
  }
  inline SampleNode* GetNext() override { return &sample_node_pool_[index_++]; }

 private:
  std::array<SampleNode, Profiler::kMaxSamples> sample_node_pool_;
};

class ResultCollection {
 public:
  virtual ~ResultCollection() = default;
  virtual const ProfileResult* Get(int index) const = 0;
};

class WorkerResultCollection final : public ResultCollection {
 public:
  WorkerResultCollection(const std::vector<WorkerProfileResult>* samples) {
    worker_samples = samples;
  }
  inline const ProfileResult* Get(int index) const override {
    return &(*worker_samples)[index];
  }

 private:
  const std::vector<WorkerProfileResult>* worker_samples;
};

class MainThreadResultCollection final : public ResultCollection {
 public:
  MainThreadResultCollection(const std::array<MainThreadProfileResult,
                                              Profiler::kMaxSamples>* samples) {
    profiler_samples = samples;
  }
  inline const ProfileResult* Get(int index) const override {
    return &(*profiler_samples)[index];
  }

 private:
  const std::array<MainThreadProfileResult, Profiler::kMaxSamples>*
      profiler_samples;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_SAMPLE_PROCESSOR_HELPERS_H_
