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

#include "core/animation/texture_transform_animation.h"

#include <cassert>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "core/animation/curve_variant_helper.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::animation {

TextureTransformAnimation::TextureTransformParameter::TextureTransformParameter(
    absl::optional<float2> in_texture_transform_offset,
    absl::optional<float> in_texture_transform_rotation,
    absl::optional<float2> in_texture_transform_scale) {
  animated_texture_transform_parameter =
      AnimatedTextureTransformParameter::None;

  if (in_texture_transform_offset.has_value()) {
    texture_transform_offset = *in_texture_transform_offset;
    animated_texture_transform_parameter = AnimatedTextureTransformParameter(
        AnimatedTextureTransformParameter::kTextureTransformOffset |
        animated_texture_transform_parameter);
  }
  if (in_texture_transform_rotation.has_value()) {
    texture_transform_rotation = *in_texture_transform_rotation;
    animated_texture_transform_parameter = AnimatedTextureTransformParameter(
        AnimatedTextureTransformParameter::kTextureTransformRotation |
        animated_texture_transform_parameter);
  }
  if (in_texture_transform_scale.has_value()) {
    texture_transform_scale = *in_texture_transform_scale;
    animated_texture_transform_parameter = AnimatedTextureTransformParameter(
        AnimatedTextureTransformParameter::kTextureTransformScale |
        animated_texture_transform_parameter);
  }
}

absl::StatusOr<TextureTransformAnimation> TextureTransformAnimation::Create(
    const schemas::TextureTransformAnimation* animation) {
  if (!animation) {
    return absl::InvalidArgumentError("texture transform animation is null");
  }

  MP_ASSIGN_OR_RETURN(CurveVariant<float2> texture_transform_offset,
                   CreateCurveVariant<float2>(animation->offset_type(),
                                              animation->offset()));

  MP_ASSIGN_OR_RETURN(CurveVariant<float> texture_transform_rotation,
                   CreateCurveVariant<float>(animation->rotation_type(),
                                             animation->rotation()));

  MP_ASSIGN_OR_RETURN(
      CurveVariant<float2> texture_transform_scale,
      CreateCurveVariant<float2>(animation->scale_type(), animation->scale()));

  return TextureTransformAnimation(
      std::move(texture_transform_offset),
      std::move(texture_transform_rotation), std::move(texture_transform_scale),
      static_cast<TexturableParameters>(animation->target()));
}

TextureTransformAnimation::TextureTransformParameter
TextureTransformAnimation::Eval(float t, Cursor* cursor) const {
  return TextureTransformAnimation::TextureTransformParameter(
      EvalValue<float2>(t, &cursor->texture_transform_offset_cursor,
                        texture_transform_offset_),
      EvalValue<float>(t, &cursor->texture_transform_rotation_cursor,
                       texture_transform_rotation_),
      EvalValue<float2>(t, &cursor->texture_transform_scale_cursor,
                        texture_transform_scale_));
}

}  // namespace imp::animation
