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

#include "core/animation/material_animation.h"

#include <cassert>
#include <cstddef>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "core/animation/curve_variant_helper.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/animation/texture_transform_animation.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::animation {

absl::StatusOr<MaterialAnimation> MaterialAnimation::Create(
    const schemas::MaterialAnimation* animation) {
  MP_ASSIGN_OR_RETURN(
      CurveVariant<float4> base_color_factor,
      CreateCurveVariant<float4>(animation->base_color_factor_type(),
                                 animation->base_color_factor()));

  MP_ASSIGN_OR_RETURN(CurveVariant<float> metallic_factor,
                   CreateCurveVariant<float>(animation->metallic_factor_type(),
                                             animation->metallic_factor()));

  MP_ASSIGN_OR_RETURN(CurveVariant<float> roughness_factor,
                   CreateCurveVariant<float>(animation->roughness_factor_type(),
                                             animation->roughness_factor()));

  MP_ASSIGN_OR_RETURN(CurveVariant<float> alpha_cutoff,
                   CreateCurveVariant<float>(animation->alpha_cutoff_type(),
                                             animation->alpha_cutoff()));

  MP_ASSIGN_OR_RETURN(CurveVariant<float3> emissive_factor,
                   CreateCurveVariant<float3>(animation->emissive_factor_type(),
                                              animation->emissive_factor()));

  MP_ASSIGN_OR_RETURN(
      CurveVariant<float> normal_texture_scale,
      CreateCurveVariant<float>(animation->normal_texture_scale_type(),
                                animation->normal_texture_scale()));

  MP_ASSIGN_OR_RETURN(
      CurveVariant<float> occlusion_texture_strength,
      CreateCurveVariant<float>(animation->occlusion_texture_strength_type(),
                                animation->occlusion_texture_strength()));

  MP_ASSIGN_OR_RETURN(
      CurveVariant<float> ior,
      CreateCurveVariant<float>(animation->ior_type(), animation->ior()));

  MP_ASSIGN_OR_RETURN(CurveVariant<float> transmission,
                   CreateCurveVariant<float>(animation->transmission_type(),
                                             animation->transmission()));

  std::vector<TextureTransformAnimation> texture_transform_animations;

  for (size_t i = 0; i < animation->texture_transforms()->size(); ++i) {
    MP_ASSIGN_OR_RETURN(auto texture_transform_animation,
                     TextureTransformAnimation::Create(
                         animation->texture_transforms()->Get(i)));
    texture_transform_animations.push_back(
        std::move(texture_transform_animation));
  }

  return MaterialAnimation(
      std::move(base_color_factor), std::move(metallic_factor),
      std::move(roughness_factor), std::move(alpha_cutoff),
      std::move(emissive_factor), std::move(normal_texture_scale),
      std::move(occlusion_texture_strength), std::move(ior),
      std::move(transmission), std::move(texture_transform_animations));
}

MaterialAnimation::MaterialParameter MaterialAnimation::Eval(
    float t, Cursor* cursor) const {
  std::vector<TextureTransformAnimation::TextureTransformParameter>
      texture_transform_parameters;
  for (const auto& animation : texture_transform_animations_) {
    auto parameter = animation.Eval(t, &cursor->texture_transform_cursor);
    parameter.texture_target = animation.GetTarget();
    texture_transform_parameters.push_back(std::move(parameter));
  }
  return MaterialAnimation::MaterialParameter(
      EvalValue<float4>(t, &cursor->base_color_factor_cursor,
                        base_color_factor_),
      EvalValue<float>(t, &cursor->metallic_factor_cursor, metallic_factor_),
      EvalValue<float>(t, &cursor->roughness_factor_cursor, roughness_factor_),
      EvalValue<float>(t, &cursor->alpha_cutoff_cursor, alpha_cutoff_),
      EvalValue<float3>(t, &cursor->emissive_factor_cursor, emissive_factor_),
      EvalValue<float>(t, &cursor->normal_texture_scale_cursor,
                       normal_texture_scale_),
      EvalValue<float>(t, &cursor->occlusion_texture_strength_cursor,
                       occlusion_texture_strength_),
      EvalValue<float>(t, &cursor->ior_cursor, ior_),
      EvalValue<float>(t, &cursor->transmission_cursor, transmission_),
      std::move(texture_transform_parameters));
}

}  // namespace imp::animation
