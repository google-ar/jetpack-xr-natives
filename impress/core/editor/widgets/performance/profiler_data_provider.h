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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_PROFILER_DATA_PROVIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_PROFILER_DATA_PROVIDER_H_

#include <thread>  // NOLINT: Need to sort by thread id.

#include "absl/strings/string_view.h"
#include "core/editor/widgets/performance/sample_processor.h"

namespace imp::editor {

class ProfilerDataProvider {
 public:
  virtual ~ProfilerDataProvider() = default;

  virtual void SelectFrame(int frame_number) = 0;
  virtual void SelectFrames(int start_frame, int end_frame) = 0;

  virtual int GetSelectedFrameStart() const = 0;
  virtual int GetSelectedFrameEnd() const = 0;
  virtual int GetSelectedFrameNumber() const = 0;

  virtual absl::string_view GetSelectedSampleName() const = 0;
  virtual void SetSelectedSampleName(absl::string_view name) = 0;

  virtual std::thread::id GetSelectedSampleThreadId() const = 0;
  virtual void SetSelectedSampleThreadId(std::thread::id thread_id) = 0;

  virtual SampleProcessor& GetSampleProcessor() = 0;
  virtual bool WereSamplesProcessedSinceLastUpdate() const = 0;
  virtual void ClearSamplesProcessed() = 0;
  virtual bool HasSelectedSampleChanged() const = 0;
  virtual void ClearSelectedSampleChanged() = 0;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_PROFILER_DATA_PROVIDER_H_
