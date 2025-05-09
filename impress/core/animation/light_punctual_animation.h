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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_LIGHT_PUNCTUAL_ANIMATION_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_LIGHT_PUNCTUAL_ANIMATION_H_

#include <cstdint>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "core/animation/curve.h"
#include "core/animation/curve_variant_helper.h"
#include "core/math/math.h"
#include "core/math/vec.h"

namespace imp::animation {

// Forward declare the flatbuffer type used to construct LightPunctualAnimation.
namespace schemas {
struct LightPunctualAnimation;
}  // namespace schemas

// Animation type which supports the light punctual animation permutation space.
class LightPunctualAnimation {
 public:
  enum AnimatedLightParameter : uint16_t {
    None = 0,
    kColor = (1 << 0),
    kIntensity = (1 << 1),
    kRange = (1 << 2),
    kSpotInnerConeAngle = (1 << 3),
    kSpotOuterConeAngle = (1 << 4)
  };
  struct LightParameter {
    LightParameter(absl::optional<float3> in_color,
                   absl::optional<float> in_intensity,
                   absl::optional<float> in_range,
                   absl::optional<float> in_spot_inner_cone_angle,
                   absl::optional<float> in_spot_outer_cone_angle) {
      animated_light_parameter = AnimatedLightParameter::None;
      if (in_color.has_value()) {
        color = *in_color;
        animated_light_parameter = AnimatedLightParameter(
            AnimatedLightParameter::kColor | animated_light_parameter);
      }
      if (in_intensity.has_value()) {
        intensity = *in_intensity;
        animated_light_parameter = AnimatedLightParameter(
            AnimatedLightParameter::kIntensity | animated_light_parameter);
      }
      if (in_range.has_value()) {
        range = *in_range;
        animated_light_parameter = AnimatedLightParameter(
            AnimatedLightParameter::kRange | animated_light_parameter);
      }
      if (in_spot_inner_cone_angle.has_value()) {
        spot_inner_cone_angle = *in_spot_inner_cone_angle;
        animated_light_parameter =
            AnimatedLightParameter(AnimatedLightParameter::kSpotInnerConeAngle |
                                   animated_light_parameter);
      }
      if (in_spot_outer_cone_angle.has_value()) {
        spot_outer_cone_angle = *in_spot_outer_cone_angle;
        animated_light_parameter =
            AnimatedLightParameter(AnimatedLightParameter::kSpotOuterConeAngle |
                                   animated_light_parameter);
      }
    }
    float3 color;
    float intensity;
    float range;
    float spot_inner_cone_angle;
    float spot_outer_cone_angle;
    AnimatedLightParameter animated_light_parameter;
  };

  struct Cursor {
    BaseCurve::Cursor color_cursor;
    BaseCurve::Cursor intensity_cursor;
    BaseCurve::Cursor range_cursor;
    BaseCurve::Cursor spot_inner_cone_angle_cursor;
    BaseCurve::Cursor spot_outer_cone_angle_cursor;
  };

  Cursor CreateCursor() { return {}; }

  static absl::StatusOr<LightPunctualAnimation> Create(
      const schemas::LightPunctualAnimation *animation);

  LightParameter Eval(float t, Cursor *cursor) const;

 protected:
  LightPunctualAnimation(CurveVariant<float3> color,
                         CurveVariant<float> intensity,
                         CurveVariant<float> range,
                         CurveVariant<float> spot_inner_cone_angle,
                         CurveVariant<float> spot_outer_cone_angle)
      : color_(std::move(color)),
        intensity_(std::move(intensity)),
        range_(std::move(range)),
        spot_inner_cone_angle_(std::move(spot_inner_cone_angle)),
        spot_outer_cone_angle_(std::move(spot_outer_cone_angle)) {}

 private:
  CurveVariant<float3> color_;
  CurveVariant<float> intensity_;
  CurveVariant<float> range_;
  CurveVariant<float> spot_inner_cone_angle_;
  CurveVariant<float> spot_outer_cone_angle_;
};
}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_LIGHT_PUNCTUAL_ANIMATION_H_
