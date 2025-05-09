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

#include "core/view/utils/frame_time.h"

#include "absl/time/time.h"
#include "core/common/trace.h"

namespace imp {

FrameTime::FrameTime(absl::Time start_time)
    : start_time_(start_time),
      last_time_(start_time),
      delta_time_(),
      accumulated_delta_time_(absl::ZeroDuration()) {}

float FrameTime::GetDeltaSeconds() const {
  return static_cast<float>(absl::ToDoubleSeconds(delta_time_));
}

float FrameTime::GetElapsedSeconds() const {
  return static_cast<float>(absl::ToDoubleSeconds(last_time_ - start_time_));
}

absl::Time FrameTime::GetLastTime() const { return last_time_; }

void FrameTime::Update(absl::Duration delta_time) {
  IMP_TRACE();
  delta_time_ = delta_time + accumulated_delta_time_;
  last_time_ += delta_time_;
  accumulated_delta_time_ = absl::ZeroDuration();
}

void FrameTime::Accumulate(absl::Duration delta_time) {
  accumulated_delta_time_ += delta_time;
}

bool FrameTime::HasAccumulatedTime() const {
  return accumulated_delta_time_ != absl::ZeroDuration();
}

}  // namespace imp
