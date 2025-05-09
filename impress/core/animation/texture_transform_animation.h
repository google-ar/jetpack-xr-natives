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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_TEXTURE_TRANSFORM_ANIMATION_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_TEXTURE_TRANSFORM_ANIMATION_H_

#include <cstdint>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "core/animation/curve.h"
#include "core/animation/curve_variant_helper.h"
#include "core/loader/provider/gltf/texturable_parameter.h"
#include "core/math/math.h"
#include "core/math/vec.h"

namespace imp::animation {

using TexturableParameters =
    loader::details::provider_gltf::TexturableParameters;

// Forward declare the flatbuffer type used to construct
// TextureTransformAnimation.
namespace schemas {
struct TextureTransformAnimation;
}  // namespace schemas

// Animation type which supports the texture_transform animation permutation
// space.
class TextureTransformAnimation {
 public:
  enum AnimatedTextureTransformParameter : uint16_t {
    None = 0,
    kTextureTransformOffset = (1 << 1),
    kTextureTransformRotation = (1 << 2),
    kTextureTransformScale = (1 << 3)
  };

  struct TextureTransformParameter {
    TextureTransformParameter(
        absl::optional<float2> in_texture_transform_offset,
        absl::optional<float> in_texture_transform_rotation,
        absl::optional<float2> in_texture_transform_scale);

    loader::details::provider_gltf::TexturableParameters texture_target;
    float2 texture_transform_offset;
    float texture_transform_rotation;
    float2 texture_transform_scale;
    AnimatedTextureTransformParameter animated_texture_transform_parameter;
  };

  struct Cursor {
    BaseCurve::Cursor texture_transform_offset_cursor;
    BaseCurve::Cursor texture_transform_rotation_cursor;
    BaseCurve::Cursor texture_transform_scale_cursor;
  };

  Cursor CreateCursor() { return {}; }

  static absl::StatusOr<TextureTransformAnimation> Create(
      const schemas::TextureTransformAnimation *animation);

  TextureTransformParameter Eval(float t, Cursor *cursor) const;

  // Returns the type of texture that this texture transform animation targets.
  TexturableParameters GetTarget() const { return target_; }

 protected:
  TextureTransformAnimation(CurveVariant<float2> texture_transform_offset,
                            CurveVariant<float> texture_transform_rotation,
                            CurveVariant<float2> texture_transform_scale,
                            TexturableParameters target)
      : texture_transform_offset_(std::move(texture_transform_offset)),
        texture_transform_rotation_(std::move(texture_transform_rotation)),
        texture_transform_scale_(std::move(texture_transform_scale)),
        target_(target) {}

 private:
  CurveVariant<float2> texture_transform_offset_;
  CurveVariant<float> texture_transform_rotation_;
  CurveVariant<float2> texture_transform_scale_;
  TexturableParameters target_;
};
}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_TEXTURE_TRANSFORM_ANIMATION_H_
