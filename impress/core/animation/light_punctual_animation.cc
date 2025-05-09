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

#include "core/animation/light_punctual_animation.h"

#include <cassert>
#include <utility>

#include "absl/status/statusor.h"
#include "core/animation/curve_variant_helper.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::animation {

absl::StatusOr<LightPunctualAnimation> LightPunctualAnimation::Create(
    const schemas::LightPunctualAnimation* animation) {
  MP_ASSIGN_OR_RETURN(
      CurveVariant<float3> color,
      CreateCurveVariant<float3>(animation->color_type(), animation->color()));

  MP_ASSIGN_OR_RETURN(CurveVariant<float> intensity,
                   CreateCurveVariant<float>(animation->intensity_type(),
                                             animation->intensity()));

  MP_ASSIGN_OR_RETURN(
      CurveVariant<float> range,
      CreateCurveVariant<float>(animation->range_type(), animation->range()));

  MP_ASSIGN_OR_RETURN(CurveVariant<float> spot_inner_cone_angle,
                   CreateCurveVariant<float>(animation->inner_cone_angle_type(),
                                             animation->inner_cone_angle()));

  MP_ASSIGN_OR_RETURN(CurveVariant<float> spot_outer_cone_angle,
                   CreateCurveVariant<float>(animation->outer_cone_angle_type(),
                                             animation->outer_cone_angle()));

  return LightPunctualAnimation(
      std::move(color), std::move(intensity), std::move(range),
      std::move(spot_inner_cone_angle), std::move(spot_outer_cone_angle));
}

LightPunctualAnimation::LightParameter LightPunctualAnimation::Eval(
    float t, Cursor* cursor) const {
  return LightPunctualAnimation::LightParameter(
      EvalValue<float3>(t, &cursor->color_cursor, color_),
      EvalValue<float>(t, &cursor->intensity_cursor, intensity_),
      EvalValue<float>(t, &cursor->range_cursor, range_),
      EvalValue<float>(t, &cursor->spot_inner_cone_angle_cursor,
                       spot_inner_cone_angle_),
      EvalValue<float>(t, &cursor->spot_outer_cone_angle_cursor,
                       spot_outer_cone_angle_));
}

}  // namespace imp::animation
