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

#include "core/animation/scalar_animation.h"

#include <utility>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/math/vec.h"

namespace imp::animation {
namespace {

template <typename T>
T EvalBezier(float t, absl::Span<const T> points, T* out_tangent = nullptr) {
  auto c1 = points[3] - (3 * points[2]) + 3 * points[1] - points[0];
  auto c2 = 3 * points[2] - 6 * points[1] + 3 * points[0];
  auto c3 = 3 * points[1] - 3 * points[0];
  auto c4 = points[0];
  auto tt = t * t;

  if (out_tangent) {
    *out_tangent = 3 * c1 * tt + 2 * c2 * t + c3;
  }

  return c1 * tt * t + c2 * tt + c3 * t + c4;
}

}  // namespace

float ScalarAnimation::Eval(absl::Duration t) {
  return curve_->Eval(absl::ToDoubleSeconds(t), &cursor_);
}

absl::StatusOr<std::unique_ptr<ScalarAnimation>>
ScalarAnimation::CreateFromKeyframes(absl::Span<const Keyframe> keyframes) {
  std::vector<ScalarCubicCurve::FrameTime> times;
  std::vector<ScalarCubicCurve::FrameValue> values;

  times.reserve(keyframes.size());
  values.reserve(keyframes.size());
  absl::c_transform(keyframes, std::back_inserter(times),
                    [](const Keyframe& keyframe) {
                      return ScalarCubicCurve::FrameTime{keyframe.t};
                    });
  absl::c_transform(
      keyframes, std::back_inserter(values),
      [](const Keyframe& keyframe) -> ScalarCubicCurve::FrameValue {
        return ScalarCubicCurve::FrameValue{
            keyframe.in_tangent, keyframe.position, keyframe.out_tangent};
      });

  return CreateInternal(std::move(times), std::move(values));
}

absl::StatusOr<std::unique_ptr<ScalarAnimation>>
ScalarAnimation::CreateFrom2DBezier(
    absl::Span<const imp::float2> control_points) {
  if (control_points.size() != 4) {
    return absl::InternalError("Wrong number of control points");
  }

  std::vector<ScalarCubicCurve::FrameTime> times;
  std::vector<ScalarCubicCurve::FrameValue> values;
  constexpr auto kSubdivisions = 24;
  times.resize(kSubdivisions);
  values.resize(kSubdivisions);

  for (int i = 0; i < kSubdivisions; i++) {
    float t = static_cast<float>(i) / (kSubdivisions - 1);
    float2 tangent;
    float2 p = EvalBezier(t, control_points, &tangent);
    times[i].t = p.x;
    if (std::fabs(tangent.x) < 1.0e-4f)
      return absl::InternalError("invalid curve");
    float slope = tangent.y / tangent.x;
    values[i].in_tangent = slope;
    values[i].out_tangent = slope;
    values[i].position = p.y;
  }

  return CreateInternal(std::move(times), std::move(values));
}

absl::StatusOr<std::unique_ptr<ScalarAnimation>>
ScalarAnimation::CreateInternal(
    std::vector<ScalarCubicCurve::FrameTime> times,
    std::vector<ScalarCubicCurve::FrameValue> values) {
  auto result = absl::WrapUnique(
      new ScalarAnimation(std::move(times), std::move(values)));
  if (!result->curve_.ok()) return result->curve_.status();
  return result;
}

ScalarAnimation::ScalarAnimation(
    std::vector<ScalarCubicCurve::FrameTime> times,
    std::vector<ScalarCubicCurve::FrameValue> values)
    : times_(std::move(times)),
      values_(std::move(values)),
      curve_(ScalarCubicCurve::Create(ScalarCubicCurve::TimeSpan(times_),
                                      ScalarCubicCurve::ValueSpan(values_))),
      cursor_() {}

}  // namespace imp::animation
