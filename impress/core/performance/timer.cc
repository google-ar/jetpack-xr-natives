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

#include "core/performance/timer.h"

#include "absl/strings/string_view.h"
#include "core/performance/profiler.h"

namespace imp {

imp::Timer::Timer(const absl::string_view name) : name_(name), stopped_(false) {
  indices_ = Profiler::AddSample(name_);
}

imp::Timer::~Timer() {
  if (!stopped_) {
    Stop();
  }
}

void imp::Timer::Stop() {
  if (stopped_) {
    return;
  }
  stopped_ = true;
  if (indices_.sample_id != -1) {
    Profiler::RecordCurrentFrameSampleEndTime(indices_.sample_frame_index,
                                              indices_.sample_id);
  }
}
}  // namespace imp
