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

#include "core/editor/widgets/performance/sample_processor.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <vector>

#include "absl/base/optimization.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/common/trace.h"
#include "core/editor/widgets/performance/sample_processor_types.h"
#include "core/performance/profiler.h"
#include "core/performance/profiler_state.h"
#include "core/performance/profiler_structs.h"

namespace imp::editor {

namespace {
struct SampleNameComparator {
  using is_transparent = void;
  bool operator()(const SampleNode* a, const SampleNode* b) const {
    return a->result->GetName() < b->result->GetName();
  }
  bool operator()(const SampleNode* a, absl::string_view b) const {
    return a->result->GetName() < b;
  }
  bool operator()(absl::string_view a, const SampleNode* b) const {
    return a < b->result->GetName();
  }
};
}  // namespace

void ProcessedSamples::SortSamples() {
  std::stable_sort(all_samples.begin(), all_samples.end(),
                   SampleNameComparator{});
}

absl::Span<SampleNode* const> ProcessedSamples::GetSamplesByName(
    absl::string_view name) const {
  auto it_pair = std::equal_range(all_samples.begin(), all_samples.end(), name,
                                  SampleNameComparator{});
  if (it_pair.first == it_pair.second) {
    return {};
  }
  return absl::MakeConstSpan(&*it_pair.first,
                             std::distance(it_pair.first, it_pair.second));
}

const ProcessedSamples& SampleProcessor::GetProcessedFrame(
    const int frame_index) const {
  return GetOrCreateState()
      .processed_samples[frame_index % MainThreadProfilerState::kMaxFrames];
}

void SampleProcessor::ProcessSamples(
    const int profiler_sample_count, NodePool& node_pool,
    ProcessedSamples& processed_samples, ResultCollection& results,
    std::vector<SampleNode*>& active_nodes_stack) {
  bool sample_has_parent;
  active_nodes_stack.clear();
  uint64_t current_sample_end_id;
  const ProfileResult* current_profiler_sample;
  size_t current_depth = 0;
  size_t max_depth = 0;

  // Effectively a call stack. Each sample is added to the stack in the loop.
  // When a sample does not take place during the start/end time of the sample
  // at the top of the stack, it gets popped and that repeats until we find a
  // sample that encapsulates the current sample.
  for (size_t i = 0; i < profiler_sample_count; ++i) {
    current_profiler_sample = results.Get(i);
    // Pop nodes that have ended before the current sample starts
    current_sample_end_id = current_profiler_sample->GetSampleEndId();
    while (!active_nodes_stack.empty() &&
           active_nodes_stack.back()->result->GetSampleEndId() <=
               current_sample_end_id) {
      active_nodes_stack.pop_back();
      current_depth--;
    }

    // Create a new node and add it as a child of the sample atop the stack.
    SampleNode* new_node = node_pool.GetNext();
    new_node->SetInitialValues(current_profiler_sample);

    // If the stack is empty, the sample has no parent.
    sample_has_parent = !active_nodes_stack.empty();

    // If the sample node has a parent, add it as a child of its parent and then
    // register it into the processed samples.
    // Otherwise, add it as a root sample in the processed samples.
    if (ABSL_PREDICT_TRUE(sample_has_parent)) {
      active_nodes_stack.back()->AddChild(new_node);
      processed_samples.AddSample(new_node);
    } else {
      processed_samples.AddRootSample(new_node);
    }
    active_nodes_stack.push_back(new_node);
    current_depth++;
    max_depth = std::max(max_depth, current_depth);
  }

  processed_samples.max_depth = max_depth;
}

void SampleProcessor::ProcessMainThreadSamples(const int frame_index) {
  IMP_TRACE();

  if (!Profiler::HasFrameRecorded(frame_index)) return;

  const int sample_index = frame_index % MainThreadProfilerState::kMaxFrames;
  ProcessedSamples& processed_samples =
      GetOrCreateState().processed_samples[sample_index];
  processed_samples.Clear();

  absl::StatusOr<int> sample_count = Profiler::GetSampleCount(frame_index);

  if (!sample_count.ok()) return;

  int count = *sample_count;

  if (count == 0) return;

  count = std::min(count, MainThreadProfilerState::kMaxSamples);

  NodePool& node_pool = GetOrCreateState().main_thread_node_pools[sample_index];
  node_pool.ResetIndex();

  absl::StatusOr<const std::array<MainThreadProfileResult,
                                  MainThreadProfilerState::kMaxSamples>*>
      profiler_samples = Profiler::GetSamples(frame_index);

  if (!profiler_samples.ok()) return;

  const std::array<MainThreadProfileResult,
                   MainThreadProfilerState::kMaxSamples>& samples =
      **profiler_samples;

  MainThreadResultCollection results(&samples);

  ProcessSamples(count, node_pool, processed_samples, results,
                 GetOrCreateState().active_nodes_stack);
  processed_samples.SortSamples();
}

void SampleProcessor::ProcessWorkerSampleList(
    const std::vector<WorkerProfileResult>& samples,
    ProcessedSamples& processed_worker_samples) {
  SampleProcessorState& state = GetOrCreateState();
  WorkerResultCollection results(&samples);
  ProcessSamples(samples.size(), state.worker_node_pool,
                 processed_worker_samples, results, state.active_nodes_stack);
  processed_worker_samples.SortSamples();
}

ProcessedWorkerSamplesMap SampleProcessor::ProcessAllWorkerThreadsSamples(
    const RawWorkerSamplesMap& raw_samples_map) {
  IMP_TRACE();

  ProcessedWorkerSamplesMap processed_worker_samples_map;
  GetOrCreateState().worker_node_pool.ResetIndex();

  for (const auto& [thread_id, samples] : raw_samples_map) {
    ProcessWorkerSampleList(samples, processed_worker_samples_map[thread_id]);
  }
  return processed_worker_samples_map;
}

ProcessedSamples SampleProcessor::ProcessWorkerThreadSamples(
    const std::vector<WorkerProfileResult>& samples) {
  IMP_TRACE();

  ProcessedSamples processed_worker_samples;
  GetOrCreateState().worker_node_pool.ResetIndex();

  ProcessWorkerSampleList(samples, processed_worker_samples);

  return processed_worker_samples;
}
}  // namespace imp::editor
