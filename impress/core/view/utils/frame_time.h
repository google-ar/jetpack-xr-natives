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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_FRAME_TIME_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_FRAME_TIME_H_

#include "absl/time/time.h"

namespace imp {

class FrameTime {
 public:
  FrameTime(absl::Time start_time);

  // Returns the number of seconds between the current frame and the previous
  // frame.
  float GetDeltaSeconds() const;

  // Returns the number of seconds between the current frame and when the view
  // was created.
  float GetElapsedSeconds() const;

  // Returns the Time of the current frame.
  absl::Time GetLastTime() const;

  // Returns true if Accumulate has been called since the last time Update was
  // called.  This means that we have pending delta time that hasn't actually
  // been applied yet.
  bool HasAccumulatedTime() const;

  // Returns the accumulated delta time, which is the pending delta time that
  // hasn't actually been applied yet.
  absl::Duration GetAccumulatedDeltaTime() const {
    return accumulated_delta_time_;
  }

  absl::Duration GetDeltaTime() const { return delta_time_; }
  absl::Duration GetElapsedTime() const { return last_time_ - start_time_; }

  void Update(absl::Duration delta_time);

  // Accumulates time without actually updating the elapsed time or delta time.
  // The accumulated time will be applied on the next call to Update.
  void Accumulate(absl::Duration delta_time);

 private:
  absl::Time start_time_;
  absl::Time last_time_;
  absl::Duration delta_time_;
  absl::Duration accumulated_delta_time_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_FRAME_TIME_H_
