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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_SCALAR_ANIMATION_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_SCALAR_ANIMATION_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/animation/curve.h"
#include "core/math/math.h"

namespace imp::animation {

namespace details {}  // namespace details

class ScalarAnimation {
 public:
  // Explicitly not const because it updates its internal cursor.
  float Eval(absl::Duration t);

  // Transition functions (e.g. fastOutSlowIn) are generally defined by a
  // quartet of 2D bezier control points.  These curves aren't evaluated against
  // a curve parameter in the traditional sense, they're compactly defining a 2D
  // function curve defined from 0 to 1.  This method converts such a curve such
  // that sampling at a given x provides the corresponding y as defined by the
  // input curve.
  static absl::StatusOr<std::unique_ptr<ScalarAnimation>> CreateFrom2DBezier(
      absl::Span<const imp::float2> control_points);

  struct Keyframe {
    float t;
    float in_tangent;
    float position;
    float out_tangent;
  };

  static absl::StatusOr<std::unique_ptr<ScalarAnimation>> CreateFromKeyframes(
      absl::Span<const Keyframe> keyframes);

 private:
  using ScalarCubicCurve = CubicCurve<float>;

  static absl::StatusOr<std::unique_ptr<ScalarAnimation>> CreateInternal(
      std::vector<ScalarCubicCurve::FrameTime> times,
      std::vector<ScalarCubicCurve::FrameValue> values);

  ScalarAnimation(std::vector<ScalarCubicCurve::FrameTime> times,
                  std::vector<ScalarCubicCurve::FrameValue> values);

  std::vector<ScalarCubicCurve::FrameTime> times_;
  std::vector<ScalarCubicCurve::FrameValue> values_;
  absl::StatusOr<ScalarCubicCurve> curve_;
  ScalarCubicCurve::Cursor cursor_;
};

}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_SCALAR_ANIMATION_H_
