/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_SAMPLE_PROCESSOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_SAMPLE_PROCESSOR_H_

#include <array>
#include <cstdint>
#include <thread>  // NOLINT: Need to sort things by thread id.
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/strings/string_view.h"
#include "core/performance/profiler.h"

namespace imp::editor {

// Stores a processed sample from the profiler.
// If the profiler contains multiple ProfileResults with the same name with
// the same parent, they will be grouped into one ProfilerSampleNode.
// This means that the result field will point to the first ProfileResult
// that was found and thus should only be used to get the name of the sample
// as its contents are otherwise not updated to reflect the grouped samples.
struct ProfilerSampleNode {
  // Pointer to the direct Profiler sample this node is based on.
  ProfileResult* result;

  // Other sample groups that took place within this sample's duration.
  ProfilerSampleNode* first_child;
  ProfilerSampleNode* next_sibling;

  // uint32 since 32 bits allows for up to 4s of frame time.
  uint32_t total_time;

  // Number of times a ProfileResult with the same name and parent was found.
  int calls;

  // uint32 since 32 bits allows for up to 4GB of memory allocated in a sample.
  uint32_t total_memory_allocated;
  uint32_t total_memory_allocations_count;

  // Sets the initial values for a new sample node, clears existing children.
  void SetInitialValues(ProfileResult* profile_result) {
    result = profile_result;
    total_time = profile_result->duration_ns;
    calls = 1;
    first_child = nullptr;
    next_sibling = nullptr;
  }

  // Adds a child node to this node's list of children.
  void AddChild(ProfilerSampleNode* child) {
    child->next_sibling = first_child;
    first_child = child;
  }
};

// Stores a hashmap of samples that can be accessed by name along with all root
// samples found.
struct ProcessedThreadSamples {
  absl::flat_hash_map<absl::string_view, std::vector<ProfilerSampleNode*>,
                      absl::Hash<absl::string_view>>
      samples_by_name;
  std::vector<ProfilerSampleNode*> sample_roots;

  inline void AddSample(ProfilerSampleNode* sample) {
    samples_by_name[sample->result->name].push_back(sample);
  }
  void AddRootSample(ProfilerSampleNode* sample) {
    AddSample(sample);
    sample_roots.push_back(sample);
  }
};

// Stores a hashmap of ProcessedThreadSamples that can be accessed by thread id.
struct ProcessedFrame {
  absl::flat_hash_map<std::thread::id, ProcessedThreadSamples,
                      absl::Hash<std::thread::id>>
      samples_by_thread_and_name;

  void AddSample(ProfilerSampleNode* sample) {
    samples_by_thread_and_name[sample->result->thread_id].AddSample(sample);
  }
  void AddRootSample(ProfilerSampleNode* sample) {
    std::thread::id thread_id = sample->result->thread_id;
    samples_by_thread_and_name[thread_id].AddRootSample(sample);
  }
};

// Processes the samples from the profiler into a tree of nodes.
// This is used by the HierarchyPanel to display the samples in a tree format.
// Also used by the FrameTimePanel to highlight its graph based on how long
// that sample took across all recorded frames.
class SampleProcessor {
 public:
  // Processes the samples for a specific frame into a tree of nodes.
  // Does nothing when called for a frame that the Profiler does not have
  // samples for.
  void ProcessSamples(int frame_index);
  // Returns a reference to the processed samples for a specific frame.
  ProcessedFrame& GetProcessedFrame(int frame_index);

 private:
  // Pool of nodes to be reused for each frame.
  std::array<std::array<ProfilerSampleNode, Profiler::kMaxSamples>,
             Profiler::kMaxFrames>
      sample_node_pool_;
  // An array of unordered maps that store the processed samples for each frame.
  // This is also sorted by thread id.
  std::array<ProcessedFrame, Profiler::kMaxFrames> processed_samples_;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_SAMPLE_PROCESSOR_H_
