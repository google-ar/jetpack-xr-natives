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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_PARAMETERS_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_PARAMETERS_H_

#include <cstdint>
#include <optional>

#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/material_library/generic_material_constants.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

namespace imp {

// A single texture parameter for a generic material w/ sampler and uv data.
struct GenericMaterialTextureParameter {
  uint64_t texture_id = 0;
  filament::TextureSampler sampler;
  mat3f uv_transform;
  bool uses_uv1 = false;
};

struct GenericMaterialParametersBaseColor {
  std::optional<GenericMaterialTextureParameter> texture;
  float4 factor = kDefaultBaseColorFactor;
};

struct GenericMaterialParametersMetallicRoughness {
  std::optional<GenericMaterialTextureParameter> texture;
  float metallic_factor = kDefaultMetallicFactor;
  float roughness_factor = kDefaultRoughnessFactor;
};

struct GenericMaterialParametersNormal {
  std::optional<GenericMaterialTextureParameter> texture;
  float factor = kDefaultNormalFactor;
};

struct GenericMaterialParametersAmbientOcclusion {
  std::optional<GenericMaterialTextureParameter> texture;
  float factor = kDefaultAmbientOcclusionFactor;
};

struct GenericMaterialParametersEmissive {
  std::optional<GenericMaterialTextureParameter> texture;
  float3 factor = kDefaultEmissiveFactor;
};

// The clearcoat parameters for a generic material.
struct GenericMaterialParametersClearcoat {
  std::optional<GenericMaterialTextureParameter> intensity_texture;
  std::optional<GenericMaterialTextureParameter> roughness_texture;
  std::optional<GenericMaterialTextureParameter> normal_texture;
  float3 factor = kDefaultClearcoatFactor;
};

// The sheen parameters for a generic material.
struct GenericMaterialParametersSheen {
  std::optional<GenericMaterialTextureParameter> color_texture;
  float3 color_factor = kDefaultSheenColorFactor;
  std::optional<GenericMaterialTextureParameter> roughness_texture;
  float roughness_factor = kDefaultSheenRoughnessFactor;
};

// The transmission parameters for a generic material.
struct GenericMaterialParametersTransmission {
  std::optional<GenericMaterialTextureParameter> texture;
  float factor = kDefaultTransmissionFactor;
};

// The refraction parameters for a generic material.
struct GenericMaterialParametersRefraction {
  float index_of_refraction = kDefaultIndexOfRefraction;
};

// The masking parameters for a generic material.
struct GenericMaterialParametersMasking {
  // The masking alpha cutoff. Default is from the glTF Spec.
  float alpha_cutoff = kDefaultAlphaCutoff;
};

// This is a mirror of schemas::GenericMaterialParameters to support
// copying the data and re-serializing for Split Engine.
struct GenericMaterialParameters {
  // Converts a GenericMaterialParameters schema to native struct.
  template <typename GenericMaterialParametersSchema>
  static GenericMaterialParameters FromFlatbuffer(
      const GenericMaterialParametersSchema& schema);
  template <typename GenericMaterialParametersSchema>
  static GenericMaterialParameters FromFlatbuffer(
      const GenericMaterialParametersSchema* schema);

  // Converts a GenericMaterialParameters struct to flatbuffer.
  template <typename SchemaCreator>
  flatbuffers::Offset<typename SchemaCreator::Schema> ToFlatbufferT(
      flatbuffers::FlatBufferBuilder& builder) const;
  flatbuffers::Offset<schemas::GenericMaterialParameters> ToFlatbuffer(
      flatbuffers::FlatBufferBuilder& builder) const;

  std::optional<GenericMaterialParametersBaseColor> base_color = std::nullopt;
  std::optional<GenericMaterialParametersMetallicRoughness> metallic_roughness =
      std::nullopt;
  std::optional<GenericMaterialParametersNormal> normal = std::nullopt;
  std::optional<GenericMaterialParametersAmbientOcclusion> ambient_occlusion =
      std::nullopt;
  std::optional<GenericMaterialParametersEmissive> emissive = std::nullopt;
  std::optional<GenericMaterialParametersClearcoat> clearcoat = std::nullopt;
  std::optional<GenericMaterialParametersSheen> sheen = std::nullopt;
  std::optional<GenericMaterialParametersTransmission> transmission =
      std::nullopt;
  std::optional<GenericMaterialParametersRefraction> refraction = std::nullopt;
  std::optional<GenericMaterialParametersMasking> masking = std::nullopt;
};

template <typename SchemaCreator>
flatbuffers::Offset<typename SchemaCreator::GenericMaterialTextureParameter>
CreateGenericMaterialTextureParameter(
    flatbuffers::FlatBufferBuilder& builder,
    const std::optional<GenericMaterialTextureParameter>& texture_parameter) {
  if (!texture_parameter.has_value()) return 0;
  typename SchemaCreator::Mat3f uv_transform(
      texture_parameter->uv_transform[0][0],
      texture_parameter->uv_transform[0][1],
      texture_parameter->uv_transform[0][2],
      texture_parameter->uv_transform[1][0],
      texture_parameter->uv_transform[1][1],
      texture_parameter->uv_transform[1][2],
      texture_parameter->uv_transform[2][0],
      texture_parameter->uv_transform[2][1],
      texture_parameter->uv_transform[2][2]);
  typename SchemaCreator::Bool uses_uv1(texture_parameter->uses_uv1);
  return SchemaCreator::CreateGenericMaterialTextureParameter(
      builder, texture_parameter->texture_id,
      CreateTextureSampler<typename SchemaCreator::TextureSamplerCreator>(
          builder, texture_parameter->sampler),
      &uv_transform, &uses_uv1);
}

template <typename SchemaCreator>
flatbuffers::Offset<typename SchemaCreator::Schema>
GenericMaterialParameters::ToFlatbufferT(
    flatbuffers::FlatBufferBuilder& builder) const {
  flatbuffers::Offset<
      typename SchemaCreator::GenericMaterialParametersBaseColor>
      base_color_offset = 0;
  if (base_color) {
    typename SchemaCreator::Float4 base_color_factor(
        base_color->factor.x, base_color->factor.y, base_color->factor.z,
        base_color->factor.w);
    base_color_offset = SchemaCreator::CreateGenericMaterialParametersBaseColor(
        builder,
        CreateGenericMaterialTextureParameter<SchemaCreator>(
            builder, base_color->texture),
        &base_color_factor);
  }
  flatbuffers::Offset<
      typename SchemaCreator::GenericMaterialParametersMetallicRoughness>
      metallic_roughness_offset = 0;
  if (metallic_roughness) {
    typename SchemaCreator::Float metallic_factor(
        metallic_roughness->metallic_factor);
    typename SchemaCreator::Float roughness_factor(
        metallic_roughness->roughness_factor);
    metallic_roughness_offset =
        SchemaCreator::CreateGenericMaterialParametersMetallicRoughness(
            builder,
            CreateGenericMaterialTextureParameter<SchemaCreator>(
                builder, metallic_roughness->texture),
            &metallic_factor, &roughness_factor);
  }

  flatbuffers::Offset<typename SchemaCreator::GenericMaterialParametersNormal>
      normal_offset = 0;
  if (normal) {
    typename SchemaCreator::Float normal_factor(normal->factor);
    normal_offset = SchemaCreator::CreateGenericMaterialParametersNormal(
        builder,
        CreateGenericMaterialTextureParameter<SchemaCreator>(builder,
                                                             normal->texture),
        &normal_factor);
  }

  flatbuffers::Offset<
      typename SchemaCreator::GenericMaterialParametersAmbientOcclusion>
      ambient_occlusion_offset = 0;
  if (ambient_occlusion) {
    typename SchemaCreator::Float ambient_occlusion_strength(
        ambient_occlusion->factor);
    ambient_occlusion_offset =
        SchemaCreator::CreateGenericMaterialParametersAmbientOcclusion(
            builder,
            CreateGenericMaterialTextureParameter<SchemaCreator>(
                builder, ambient_occlusion->texture),
            &ambient_occlusion_strength);
  }

  flatbuffers::Offset<typename SchemaCreator::GenericMaterialParametersEmissive>
      emissive_offset = 0;
  if (emissive) {
    typename SchemaCreator::Float3 emissive_factor(
        emissive->factor.x, emissive->factor.y, emissive->factor.z);
    emissive_offset = SchemaCreator::CreateGenericMaterialParametersEmissive(
        builder,
        CreateGenericMaterialTextureParameter<SchemaCreator>(builder,
                                                             emissive->texture),
        &emissive_factor);
  }

  flatbuffers::Offset<
      typename SchemaCreator::GenericMaterialParametersClearcoat>
      clearcoat_offset = 0;
  if (clearcoat) {
    typename SchemaCreator::Float3 clearcoat_factor(
        clearcoat->factor.x, clearcoat->factor.y, clearcoat->factor.z);
    clearcoat_offset = SchemaCreator::CreateGenericMaterialParametersClearcoat(
        builder,
        CreateGenericMaterialTextureParameter<SchemaCreator>(
            builder, clearcoat->intensity_texture),
        CreateGenericMaterialTextureParameter<SchemaCreator>(
            builder, clearcoat->normal_texture),
        CreateGenericMaterialTextureParameter<SchemaCreator>(
            builder, clearcoat->roughness_texture),
        &clearcoat_factor);
  }
  flatbuffers::Offset<typename SchemaCreator::GenericMaterialParametersSheen>
      sheen_offset = 0;
  if (sheen) {
    typename SchemaCreator::Float3 sheen_color_factor(
        sheen->color_factor.x, sheen->color_factor.y, sheen->color_factor.z);
    typename SchemaCreator::Float sheen_roughness_factor(
        sheen->roughness_factor);
    sheen_offset = SchemaCreator::CreateGenericMaterialParametersSheen(
        builder,
        CreateGenericMaterialTextureParameter<SchemaCreator>(
            builder, sheen->color_texture),
        &sheen_color_factor,
        CreateGenericMaterialTextureParameter<SchemaCreator>(
            builder, sheen->roughness_texture),
        &sheen_roughness_factor);
  }
  flatbuffers::Offset<
      typename SchemaCreator::GenericMaterialParametersTransmission>
      transmission_offset = 0;
  if (transmission) {
    typename SchemaCreator::Float transmission_factor(transmission->factor);
    transmission_offset =
        SchemaCreator::CreateGenericMaterialParametersTransmission(
            builder,
            CreateGenericMaterialTextureParameter<SchemaCreator>(
                builder, transmission->texture),
            &transmission_factor);
  }
  flatbuffers::Offset<
      typename SchemaCreator::GenericMaterialParametersRefraction>
      refraction_offset = 0;
  if (refraction) {
    typename SchemaCreator::Float refraction_factor(
        refraction->index_of_refraction);
    refraction_offset =
        SchemaCreator::CreateGenericMaterialParametersRefraction(
            builder, &refraction_factor);
  }
  flatbuffers::Offset<typename SchemaCreator::GenericMaterialParametersMasking>
      masking_offset = 0;
  if (masking) {
    typename SchemaCreator::Float alpha_cutoff_factor(masking->alpha_cutoff);
    masking_offset = SchemaCreator::CreateGenericMaterialParametersMasking(
        builder, &alpha_cutoff_factor);
  }
  return SchemaCreator::CreateGenericMaterialParameters(
      builder, base_color_offset, metallic_roughness_offset, normal_offset,
      ambient_occlusion_offset, emissive_offset, clearcoat_offset, sheen_offset,
      transmission_offset, refraction_offset, masking_offset);
}

template <typename GenericMaterialTextureParameterSchema>
GenericMaterialTextureParameter FromTextureParameterFlatbuffer(
    const GenericMaterialTextureParameterSchema& texture_parameter) {
  return GenericMaterialTextureParameter{
      .texture_id = texture_parameter.texture(),
      .sampler = (texture_parameter.sampler()
                      ? ConvertSampler(texture_parameter.sampler()).value()
                      : filament::TextureSampler()),
      .uv_transform =
          texture_parameter.uv_transform()
              ? FromMat3fFlatbuffer(*texture_parameter.uv_transform())
              : mat3f(),
      .uses_uv1 = texture_parameter.uses_uv1()
                      ? FromBoolFlatbuffer(*texture_parameter.uses_uv1())
                      : false,
  };
}

template <typename GenericMaterialTextureParameterSchema>
std::optional<GenericMaterialTextureParameter> FromTextureParameterFlatbuffer(
    const GenericMaterialTextureParameterSchema* texture_parameter) {
  if (texture_parameter == nullptr) return std::nullopt;

  return FromTextureParameterFlatbuffer(*texture_parameter);
}

template <typename GenericMaterialParametersBaseColorSchema>
GenericMaterialParametersBaseColor FromBaseColorFlatbuffer(
    const GenericMaterialParametersBaseColorSchema& base_color) {
  return GenericMaterialParametersBaseColor{
      .texture = FromTextureParameterFlatbuffer(base_color.texture()),
      .factor = base_color.factor() ? FromFloat4Flatbuffer(*base_color.factor())
                                    : kDefaultBaseColorFactor,
  };
}

template <typename GenericMaterialParametersMetallicRoughnessSchema>
GenericMaterialParametersMetallicRoughness FromMetallicRoughnessFlatbuffer(
    const GenericMaterialParametersMetallicRoughnessSchema&
        metallic_roughness) {
  return GenericMaterialParametersMetallicRoughness{
      .texture = FromTextureParameterFlatbuffer(metallic_roughness.texture()),
      .metallic_factor =
          metallic_roughness.metallic_factor()
              ? FromFloatFlatbuffer(*metallic_roughness.metallic_factor())
              : kDefaultMetallicFactor,
      .roughness_factor =
          metallic_roughness.roughness_factor()
              ? FromFloatFlatbuffer(*metallic_roughness.roughness_factor())
              : kDefaultRoughnessFactor,
  };
}

template <typename GenericMaterialParametersNormalSchema>
GenericMaterialParametersNormal FromNormalFlatbuffer(
    const GenericMaterialParametersNormalSchema& normal) {
  return GenericMaterialParametersNormal{
      .texture = FromTextureParameterFlatbuffer(normal.texture()),
      .factor = normal.factor() ? FromFloatFlatbuffer(*normal.factor())
                                : kDefaultNormalFactor,
  };
}

template <typename GenericMaterialParametersAmbientOcclusionSchema>
GenericMaterialParametersAmbientOcclusion FromAmbientOcclusionFlatbuffer(
    const GenericMaterialParametersAmbientOcclusionSchema& ambient_occlusion) {
  return GenericMaterialParametersAmbientOcclusion{
      .texture = FromTextureParameterFlatbuffer(ambient_occlusion.texture()),
      .factor = ambient_occlusion.factor()
                    ? FromFloatFlatbuffer(*ambient_occlusion.factor())
                    : kDefaultAmbientOcclusionFactor,
  };
}

template <typename GenericMaterialParametersEmissiveSchema>
GenericMaterialParametersEmissive FromEmissiveFlatbuffer(
    const GenericMaterialParametersEmissiveSchema& emissive) {
  return GenericMaterialParametersEmissive{
      .texture = FromTextureParameterFlatbuffer(emissive.texture()),
      .factor = emissive.factor() ? FromFloat3Flatbuffer(*emissive.factor())
                                  : kDefaultEmissiveFactor,
  };
}

template <typename GenericMaterialParametersClearcoatSchema>
GenericMaterialParametersClearcoat FromClearcoatFlatbuffer(
    const GenericMaterialParametersClearcoatSchema& clearcoat) {
  return GenericMaterialParametersClearcoat{
      .intensity_texture =
          FromTextureParameterFlatbuffer(clearcoat.intensity_texture()),
      .roughness_texture =
          FromTextureParameterFlatbuffer(clearcoat.roughness_texture()),
      .normal_texture =
          FromTextureParameterFlatbuffer(clearcoat.normal_texture()),
      .factor = clearcoat.factor() ? FromFloat3Flatbuffer(*clearcoat.factor())
                                   : kDefaultClearcoatFactor,
  };
}

template <typename GenericMaterialParametersSheenSchema>
GenericMaterialParametersSheen FromSheenFlatbuffer(
    const GenericMaterialParametersSheenSchema& sheen) {
  return GenericMaterialParametersSheen{
      .color_texture = FromTextureParameterFlatbuffer(sheen.color_texture()),
      .color_factor = sheen.color_factor()
                          ? FromFloat3Flatbuffer(*sheen.color_factor())
                          : kDefaultSheenColorFactor,
      .roughness_texture =
          FromTextureParameterFlatbuffer(sheen.roughness_texture()),
      .roughness_factor = sheen.roughness_factor()
                              ? FromFloatFlatbuffer(*sheen.roughness_factor())
                              : kDefaultSheenRoughnessFactor,
  };
}

template <typename GenericMaterialParametersTransmissionSchema>
GenericMaterialParametersTransmission FromTransmissionFlatbuffer(
    const GenericMaterialParametersTransmissionSchema& transmission) {
  return GenericMaterialParametersTransmission{
      .texture = FromTextureParameterFlatbuffer(transmission.texture()),
      .factor = transmission.factor()
                    ? FromFloatFlatbuffer(*transmission.factor())
                    : kDefaultTransmissionFactor,
  };
}

template <typename GenericMaterialParametersRefractionSchema>
GenericMaterialParametersRefraction FromRefractionFlatbuffer(
    const GenericMaterialParametersRefractionSchema& refraction) {
  return GenericMaterialParametersRefraction{
      .index_of_refraction =
          refraction.index_of_refraction()
              ? FromFloatFlatbuffer(*refraction.index_of_refraction())
              : kDefaultIndexOfRefraction,
  };
}

template <typename GenericMaterialParametersMaskingSchema>
GenericMaterialParametersMasking FromMaskingFlatbuffer(
    const GenericMaterialParametersMaskingSchema& masking) {
  return GenericMaterialParametersMasking{
      .alpha_cutoff = masking.alpha_cutoff()
                          ? FromFloatFlatbuffer(*masking.alpha_cutoff())
                          : kDefaultAlphaCutoff,
  };
}

template <typename GenericMaterialParametersSchema>
GenericMaterialParameters GenericMaterialParameters::FromFlatbuffer(
    const GenericMaterialParametersSchema& schema) {
  GenericMaterialParameters parameters;
  if (schema.base_color()) {
    parameters.base_color = FromBaseColorFlatbuffer(*schema.base_color());
  }
  if (schema.metallic_roughness()) {
    parameters.metallic_roughness =
        FromMetallicRoughnessFlatbuffer(*schema.metallic_roughness());
  }
  if (schema.normal()) {
    parameters.normal = FromNormalFlatbuffer(*schema.normal());
  }
  if (schema.ambient_occlusion()) {
    parameters.ambient_occlusion =
        FromAmbientOcclusionFlatbuffer(*schema.ambient_occlusion());
  }
  if (schema.emissive()) {
    parameters.emissive = FromEmissiveFlatbuffer(*schema.emissive());
  }
  if (schema.clearcoat()) {
    parameters.clearcoat = FromClearcoatFlatbuffer(*schema.clearcoat());
  }
  if (schema.sheen()) {
    parameters.sheen = FromSheenFlatbuffer(*schema.sheen());
  }
  if (schema.transmission()) {
    parameters.transmission =
        FromTransmissionFlatbuffer(*schema.transmission());
  }
  if (schema.refraction()) {
    parameters.refraction = FromRefractionFlatbuffer(*schema.refraction());
  }
  if (schema.masking()) {
    parameters.masking = FromMaskingFlatbuffer(*schema.masking());
  } else {
    parameters.masking.emplace();
    parameters.masking->alpha_cutoff = kDefaultAlphaCutoff;
  }
  return parameters;
}

template <typename GenericMaterialParametersSchema>
GenericMaterialParameters GenericMaterialParameters::FromFlatbuffer(
    const GenericMaterialParametersSchema* schema) {
  if (!schema) return GenericMaterialParameters();
  return FromFlatbuffer(*schema);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_PARAMETERS_H_
