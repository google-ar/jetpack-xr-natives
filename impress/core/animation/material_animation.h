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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_MATERIAL_ANIMATION_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_MATERIAL_ANIMATION_H_

#include <cstdint>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "core/animation/curve.h"
#include "core/animation/curve_variant_helper.h"
#include "core/animation/texture_transform_animation.h"
#include "core/math/math.h"
#include "core/math/vec.h"

namespace imp::animation {

// Forward declare the flatbuffer type used to construct MaterialAnimation.
namespace schemas {
struct MaterialAnimation;
}  // namespace schemas

// Animation type which supports the material animation permutation space.
class MaterialAnimation {
 public:
  enum AnimatedMaterialParameter : uint16_t {
    None = 0,
    kBaseColorFactor = (1 << 0),
    kMetallicFactor = (1 << 1),
    kRoughnessFactor = (1 << 2),
    kAlphaCutoff = (1 << 3),
    kEmissiveFactor = (1 << 4),
    kNormalTextureScale = (1 << 5),
    kOcclusionTextureStrength = (1 << 6),
    kIor = (1 << 7),
    kTransmission = (1 << 8)
  };
  struct MaterialParameter {
    MaterialParameter(
        absl::optional<float4> in_base_color_factor,
        absl::optional<float> in_metallic_factor,
        absl::optional<float> in_roughness_factor,
        absl::optional<float> in_alpha_cutoff,
        absl::optional<float3> in_emissive_factor,
        absl::optional<float> in_normal_texture_scale,
        absl::optional<float> in_occlusion_texture_strength,
        absl::optional<float> in_ior, absl::optional<float> in_transmission,
        std::vector<TextureTransformAnimation::TextureTransformParameter>
            in_texture_transform_parameters)
        : texture_transform_parameters(
              std::move(in_texture_transform_parameters)) {
      animated_material_parameter = AnimatedMaterialParameter::None;
      if (in_base_color_factor.has_value()) {
        base_color_factor = *in_base_color_factor;
        animated_material_parameter = AnimatedMaterialParameter(
            AnimatedMaterialParameter::kBaseColorFactor |
            animated_material_parameter);
      }
      if (in_metallic_factor.has_value()) {
        metallic_factor = *in_metallic_factor;
        animated_material_parameter = AnimatedMaterialParameter(
            AnimatedMaterialParameter::kMetallicFactor |
            animated_material_parameter);
      }
      if (in_roughness_factor.has_value()) {
        roughness_factor = *in_roughness_factor;
        animated_material_parameter = AnimatedMaterialParameter(
            AnimatedMaterialParameter::kRoughnessFactor |
            animated_material_parameter);
      }
      if (in_alpha_cutoff.has_value()) {
        alpha_cutoff = *in_alpha_cutoff;
        animated_material_parameter =
            AnimatedMaterialParameter(AnimatedMaterialParameter::kAlphaCutoff |
                                      animated_material_parameter);
      }
      if (in_emissive_factor.has_value()) {
        emissive_factor = *in_emissive_factor;
        animated_material_parameter = AnimatedMaterialParameter(
            AnimatedMaterialParameter::kEmissiveFactor |
            animated_material_parameter);
      }
      if (in_normal_texture_scale.has_value()) {
        normal_texture_scale = *in_normal_texture_scale;
        animated_material_parameter = AnimatedMaterialParameter(
            AnimatedMaterialParameter::kNormalTextureScale |
            animated_material_parameter);
      }
      if (in_occlusion_texture_strength.has_value()) {
        occlusion_texture_strength = *in_occlusion_texture_strength;
        animated_material_parameter = AnimatedMaterialParameter(
            AnimatedMaterialParameter::kOcclusionTextureStrength |
            animated_material_parameter);
      }
      if (in_ior.has_value()) {
        ior = *in_ior;
        animated_material_parameter = AnimatedMaterialParameter(
            AnimatedMaterialParameter::kIor | animated_material_parameter);
      }
      if (in_transmission.has_value()) {
        transmission = *in_transmission;
        animated_material_parameter =
            AnimatedMaterialParameter(AnimatedMaterialParameter::kTransmission |
                                      animated_material_parameter);
      }
    }
    float4 base_color_factor;
    float metallic_factor;
    float roughness_factor;
    float alpha_cutoff;
    float3 emissive_factor;
    float normal_texture_scale;
    float occlusion_texture_strength;
    float ior;
    float transmission;
    AnimatedMaterialParameter animated_material_parameter;
    std::vector<TextureTransformAnimation::TextureTransformParameter>
        texture_transform_parameters;
  };

  struct Cursor {
    BaseCurve::Cursor base_color_factor_cursor;
    BaseCurve::Cursor metallic_factor_cursor;
    BaseCurve::Cursor roughness_factor_cursor;
    BaseCurve::Cursor alpha_cutoff_cursor;
    BaseCurve::Cursor emissive_factor_cursor;
    BaseCurve::Cursor normal_texture_scale_cursor;
    BaseCurve::Cursor occlusion_texture_strength_cursor;
    BaseCurve::Cursor ior_cursor;
    BaseCurve::Cursor transmission_cursor;
    TextureTransformAnimation::Cursor texture_transform_cursor;
  };

  Cursor CreateCursor() { return {}; }

  static absl::StatusOr<MaterialAnimation> Create(
      const schemas::MaterialAnimation *animation);

  MaterialParameter Eval(float t, Cursor *cursor) const;

 protected:
  MaterialAnimation(
      CurveVariant<float4> base_color_factor,
      CurveVariant<float> metallic_factor, CurveVariant<float> roughness_factor,
      CurveVariant<float> alpha_cutoff, CurveVariant<float3> emissive_factor,
      CurveVariant<float> normal_texture_scale,
      CurveVariant<float> occlusion_texture_strength, CurveVariant<float> ior,
      CurveVariant<float> transmission,
      std::vector<TextureTransformAnimation> texture_transform_animations)
      : base_color_factor_(std::move(base_color_factor)),
        metallic_factor_(std::move(metallic_factor)),
        roughness_factor_(std::move(roughness_factor)),
        alpha_cutoff_(std::move(alpha_cutoff)),
        emissive_factor_(std::move(emissive_factor)),
        normal_texture_scale_(std::move(normal_texture_scale)),
        occlusion_texture_strength_(std::move(occlusion_texture_strength)),
        ior_(std::move(ior)),
        transmission_(std::move(transmission)),
        texture_transform_animations_(std::move(texture_transform_animations)) {
  }

 private:
  CurveVariant<float4> base_color_factor_;
  CurveVariant<float> metallic_factor_;
  CurveVariant<float> roughness_factor_;
  CurveVariant<float> alpha_cutoff_;
  CurveVariant<float3> emissive_factor_;
  CurveVariant<float> normal_texture_scale_;
  CurveVariant<float> occlusion_texture_strength_;
  CurveVariant<float> ior_;
  CurveVariant<float> transmission_;
  std::vector<TextureTransformAnimation> texture_transform_animations_;
};
}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_MATERIAL_ANIMATION_H_
