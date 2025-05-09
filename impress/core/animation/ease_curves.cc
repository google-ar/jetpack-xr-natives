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

#include "core/animation/ease_curves.h"

#include <array>
#include <memory>

#include "absl/log/check.h"
#include "absl/types/span.h"
#include "core/animation/scalar_animation.h"
#include "core/math/vec.h"

namespace imp::animation {

namespace {

// Extracts ptr from unique_ptr for static storage.

ScalarAnimation* ExtractAnimation(
    const std::array<imp::float2, 4>& control_points) {
  absl::StatusOr<std::unique_ptr<ScalarAnimation>> animation =
      ScalarAnimation::CreateFrom2DBezier(
          absl::MakeConstSpan(control_points.begin(), control_points.end()));
  
  return animation->release();
}

}  // namespace

float FastOutSlowIn(float t) {
  // Control points are based on values from
  // https://material.io/design/motion/speed.html#easing
  static constexpr std::array<imp::float2, 4> fastout_slowin_control_points = {
      imp::kZero2, imp::float2{.4f, 0.0f}, imp::float2{.2f, 1.0f}, imp::kOne2};

  static ScalarAnimation* animation =
      ExtractAnimation(fastout_slowin_control_points);

  return animation->Eval(absl::Seconds(t));
}

float LinearOutSlowIn(float t) {
  // Control points are based on values from
  // https://material.io/design/motion/speed.html#easing
  static constexpr std::array<imp::float2, 4> linearout_slowin_control_points{
      imp::kZero2, imp::float2{0.0001f, 0.0f}, imp::float2{.2f, 1.0f},
      imp::kOne2};

  static ScalarAnimation* animation =
      ExtractAnimation(linearout_slowin_control_points);

  return animation->Eval(absl::Seconds(t));
}

float FastOutLinearIn(float t) {
  // Control points are based on values from
  // https://material.io/design/motion/speed.html#easing
  static constexpr std::array<imp::float2, 4> fastout_linearin_control_points{
      imp::kZero2, imp::float2{0.4f, 0.0f}, imp::float2{.9999f, 1.0f},
      imp::kOne2};

  static ScalarAnimation* animation =
      ExtractAnimation(fastout_linearin_control_points);
  return animation->Eval(absl::Seconds(t));
}

}  // namespace imp::animation
