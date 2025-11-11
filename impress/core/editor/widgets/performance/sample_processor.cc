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
#include <stack>
#include <thread>  // NOLINT: Need to sort things by thread id.

#include "absl/base/optimization.h"
#include "absl/container/flat_hash_map.h"
#include "core/common/trace.h"
#include "core/performance/profiler.h"

namespace imp::editor {

ProcessedFrame& SampleProcessor::GetProcessedFrame(int frame_index) {
  return processed_samples_[frame_index % Profiler::kMaxFrames];
}

void SampleProcessor::ProcessSamples(int frame_index) {
  IMP_TRACE();

  if (!Profiler::HasFrameRecorded(frame_index)) return;

  int sample_index = frame_index % Profiler::kMaxFrames;

  processed_samples_[sample_index].samples_by_thread_and_name.clear();

  // Returns 0 if the frame is not currently available or contains no samples.
  int profiler_sample_count = Profiler::GetSampleCount(frame_index);
  if (profiler_sample_count <= 0) {
    return;
  }

  profiler_sample_count =
      std::min(profiler_sample_count, Profiler::kMaxSamples);

  std::array<ProfileResult, Profiler::kMaxSamples>& profiler_samples =
      Profiler::GetSamples(frame_index);

  // Effectively a call stack. Each sample is added to the stack in the loop.
  // When a sample does not take place during the start/end time of the sample
  // at the top of the stack, it gets popped and that repeats until we find a
  // sample that encapsulates the current sample.
  absl::flat_hash_map<std::thread::id, std::stack<ProfilerSampleNode*>>
      active_nodes;
  std::thread::id last_thread_id;
  bool sample_has_parent;
  std::stack<ProfilerSampleNode*>* active_nodes_for_thread;

  for (size_t i = 0; i < profiler_sample_count; ++i) {
    // Only do a map lookup if the thread id has changed for performance.
    // Otherwise we still have a reference to the stack we need to use.
    std::thread::id new_thread_id = profiler_samples[i].thread_id;
    if (ABSL_PREDICT_FALSE(new_thread_id != last_thread_id)) {
      active_nodes_for_thread = &active_nodes[new_thread_id];
      last_thread_id = new_thread_id;
    }

    // Pop nodes that have ended before the current sample starts
    while (!active_nodes_for_thread->empty() &&
           active_nodes_for_thread->top()->result->sample_end_id <=
               profiler_samples[i].sample_end_id) {
      active_nodes_for_thread->pop();
    }

    // Create a new node and add it as a child of the sample atop the stack.
    ProfilerSampleNode* new_node = &sample_node_pool_[sample_index][i];
    new_node->SetInitialValues(&profiler_samples[i]);

    // If the stack is empty, the sample has no parent.
    sample_has_parent = !active_nodes_for_thread->empty();

    // If the sample node has a parent, add it as a child of its parent and then
    // register it into the processed samples.
    // Otherwise, add it as a root sample in the processed samples.
    if (ABSL_PREDICT_TRUE(sample_has_parent)) {
      active_nodes_for_thread->top()->AddChild(new_node);
      processed_samples_[sample_index].AddSample(new_node);
    } else {
      processed_samples_[sample_index].AddRootSample(new_node);
    }
    active_nodes_for_thread->push(new_node);
  }
}

}  // namespace imp::editor
