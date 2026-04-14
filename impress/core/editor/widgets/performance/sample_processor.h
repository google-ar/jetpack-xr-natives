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
#include <thread>  // NOLINT: Need to sort things by thread id.
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/performance/profiler.h"
#include "core/performance/profiler_structs.h"

namespace imp::editor {

using RawWorkerSamplesMap =
    absl::flat_hash_map<std::thread::id, std::vector<WorkerProfileResult>>;
using ProcessedWorkerSamplesMap =
    absl::flat_hash_map<std::thread::id, ProcessedSamples>;

// Processes the samples from the profiler into a tree of nodes.
// This is used by the HierarchyPanel to display the samples in a tree format.
// Also used by the FrameTimePanel to highlight its graph based on how long
// that sample took across all recorded frames.
class SampleProcessor {
 public:
  // Processes the samples for a specific frame into a tree of nodes.
  // Does nothing when called for a frame that the Profiler does not have
  // samples for.
  void ProcessMainThreadSamples(int frame_index);
  ProcessedSamples ProcessWorkerThreadSamples(
      const std::vector<WorkerProfileResult>& samples);
  ProcessedWorkerSamplesMap ProcessAllWorkerThreadsSamples(
      const RawWorkerSamplesMap& raw_samples_map);
  // Returns a reference to the processed samples for a specific frame.
  const ProcessedSamples& GetProcessedFrame(int frame_index) const;

 private:
  void ProcessSamples(int profiler_sample_count, NodePool& node_pool,
                      ProcessedSamples& processed_samples,
                      ResultCollection& results);

  std::array<MainThreadNodePool, Profiler::kMaxFrames> main_thread_node_pools_;
  WorkerNodePool worker_node_pool_;

  // Pool of processed samples, one per frame.
  std::array<ProcessedSamples, Profiler::kMaxFrames> processed_samples_;
  void ProcessWorkerSampleList(const std::vector<WorkerProfileResult>& samples,
                               ProcessedSamples& processed_worker_samples);
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_SAMPLE_PROCESSOR_H_
