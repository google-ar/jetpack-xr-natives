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

#include "core/animation/curve.h"

namespace imp::animation {

absl::Status BaseCurve::CheckTimes(TimeSpan times, size_t values_size) {
  if (times.empty() || times.size() != values_size) {
    return absl::InternalError("times and values must match and be non-empty");
  }
  for (size_t i = 1, c = times.size(); i < c; ++i) {
    if (times[Cursor::At(i)].t < times[Cursor::At(i - 1)].t) {
      return absl::InternalError("times must be monotonically increasing.");
    }
  }
  return absl::OkStatus();
}

BaseCurve::Cursor BaseCurve::UpdateCursor(float t, Cursor cursor) const {
  const auto end_cursor = Cursor::At(times_.size());
  if (!cursor) {
    // Our cursor is uninitialized; if we're not actually ahead of the first
    // keyframe, initialize.
    return (t <= times_.front().t) ? cursor : CursorAt(t);
  }

  float expected_lower_bound =
      (cursor == end_cursor) ? times_.back().t : times_[cursor].t;
  if (t < expected_lower_bound) {
    // We looped back around; Recalculate.
    return (t <= times_.front().t) ? Cursor{} : CursorAt(t);
  }
  if (cursor == end_cursor || t < times_[cursor + 1].t) {
    // Cursor value is correct.
    return cursor;
  }
  // We advanced past our cursor segment.  update it.
  return CursorAt(t, cursor + 1);
}

BaseCurve::BaseCurve(TimeSpan times) : times_(std::move(times)) {}

BaseCurve::Cursor BaseCurve::CursorAt(float t, Cursor start) const {
  // It isn't legal to call us with a t earlier than our start time.
  assert(t >= times_.front().t);
  auto it =
      std::upper_bound(times_.begin() + int16_t{start}, times_.end(), t,
                       [](float t, const FrameTime& f) { return t < f.t; });
  if (it == times_.end()) {
    // Point to one-past-the-end.
    return *times_.EndId<Cursor>();
  }
  assert(it != times_.begin());
  return times_.IdOf(*it) - 1;
}

}  // namespace imp::animation
