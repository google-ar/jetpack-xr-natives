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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_CURVE_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_CURVE_H_

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/animation/curve_values.h"
#include "core/common/paired_span.h"
#include "core/common/typed_id.h"
#include "core/common/typed_span.h"
#include "core/math/math.h"

namespace imp::animation {

// BaseCurve separates cursor update (i.e. finding the correct key/pair given an
// input t) from the bespoke template instantiations of individual curve/value
// types (Step/Linear/Cubic x float/float3/quatf).
class BaseCurve {
 public:
  struct FrameTime {
    float t;
  };
  // A cursor into the key array is represented with a TypedId.  Times are
  // stored in a TypedVector, and values are stored in a PairedVector, so a
  // Cursor is required in order to index either collection.
  using Cursor = TypedId<const FrameTime, int16_t>;
  using TimeSpan = TypedSpan<const FrameTime>;

  // A Cursor with an internal value N means:
  // N==-1 (default for Cursor): ahead of the first keyframe pair.
  // N==size() (i.e. end()):    past the last keyframe pair.  We could use
  //                            size-1 as it is similarly invalid.
  // Otherwise:                 Curve value is between times N and N+1.
  Cursor UpdateCursor(float t, Cursor cursor) const;

 protected:
  explicit BaseCurve(TimeSpan times);

  // Ensure times are compatible for playback (e.g. times must be monotonically
  // increasing).
  static absl::Status CheckTimes(TimeSpan times, size_t values_size);

  Cursor CursorAt(float t, Cursor start = Cursor::At(1)) const;

  TimeSpan times_;
};

template <typename FrameValueT>
class Curve final : BaseCurve {
 public:
  using BaseCurve::Cursor;
  using BaseCurve::FrameTime;
  using BaseCurve::TimeSpan;

  using FrameValue = FrameValueT;
  using ValueSpan = PairedSpan<const FrameValue, Cursor::ReferredType>;
  using ValueType = typename FrameValueT::ValueType;

  static absl::StatusOr<Curve> Create(TimeSpan times, ValueSpan values);

  Curve(const Curve&) = delete;
  Curve& operator=(const Curve& rhs) = delete;
  Curve(Curve&& rhs) = default;
  Curve& operator=(Curve&& rhs) = default;

  ValueType Eval(float t, Cursor* cursor_ptr) const;
  float GetMaxT() const;

 private:
  Curve(TimeSpan times, ValueSpan values);

  ValueSpan values_;
};

// Helper aliases for CurveType.
template <typename V>
using StepFrameValue = CurveFrameValue<CurveType::kStep, V>;
template <typename V>
using StepCurve = Curve<StepFrameValue<V>>;
template <typename V>
using LinearFrameValue = CurveFrameValue<CurveType::kLinear, V>;
template <typename V>
using LinearCurve = Curve<LinearFrameValue<V>>;
template <typename V>
using CubicFrameValue = CurveFrameValue<CurveType::kCubicSpline, V>;
template <typename V>
using CubicCurve = Curve<CubicFrameValue<V>>;

// Helper aliases for EasingCurveType.
template <typename V>
using EaseInSineCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseInSine, V>>;
template <typename V>
using EaseOutSineCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseOutSine, V>>;
template <typename V>
using EaseInOutSineCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseInOutSine, V>>;
template <typename V>
using EaseInQuadCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseInQuad, V>>;
template <typename V>
using EaseOutQuadCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseOutQuad, V>>;
template <typename V>
using EaseInOutQuadCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseInOutQuad, V>>;
template <typename V>
using EaseInCubicCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseInCubic, V>>;
template <typename V>
using EaseOutCubicCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseOutCubic, V>>;
template <typename V>
using EaseInOutCubicCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseInOutCubic, V>>;
template <typename V>
using EaseInBackCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseInBack, V>>;
template <typename V>
using EaseOutBackCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseOutBack, V>>;
template <typename V>
using EaseInOutBackCurve =
    Curve<EasingFrameValue<EasingCurveType::kEaseInOutBack, V>>;

template <typename FrameValueT>
Curve<FrameValueT>::Curve(TimeSpan times, ValueSpan values)
    : BaseCurve(std::move(times)), values_(std::move(values)) {}

template <typename FrameValueT>
typename Curve<FrameValueT>::ValueType Curve<FrameValueT>::Eval(
    float t, Cursor* cursor_ptr) const {
  Cursor& cursor = *cursor_ptr;
  cursor = UpdateCursor(t, cursor);
  if (!cursor) {
    // Ahead of first segment.
    return values_.front().position;
  } else if (cursor == *times_.EndId<Cursor>()) {
    // Past the last segment
    return values_.back().position;
  }

  // Interpolate at our cursor.
  assert(times_.IsValid(cursor + 1));
  const float start_t = times_[cursor].t;
  const float end_t = times_[cursor + 1].t;
  assert(t >= start_t && t < end_t);
  const FrameValue& start_v = values_[cursor];
  const FrameValue& end_v = values_[cursor + 1];
  const float duration = (end_t - start_t);
  float sub_t = (t - start_t) / duration;
  return FrameValue::Interpolate(start_v, end_v, sub_t, duration);
}

template <typename FrameValueT>
absl::StatusOr<Curve<FrameValueT>> Curve<FrameValueT>::Create(
    TimeSpan times, ValueSpan values) {
  if (auto status = CheckTimes(times, values.size()); !status.ok()) {
    return status;
  }
  return Curve(times, values);
}

template <typename FrameValueT>
float Curve<FrameValueT>::GetMaxT() const {
  return times_.back().t;
}

}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_CURVE_H_
