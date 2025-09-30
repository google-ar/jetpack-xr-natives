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

#include "core/material_library/generic_material_schema_provider.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/schemas/math_generated.h"
#include "core/common/schemas/render_generated.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_attribute.h"
#include "core/loader/provider/gltf/gltf_geometry.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/loader/provider/gltf/gltf_texture.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/generic_material_constants.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_param_value.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/math/flatbuffer_support.h"
#include "core/math/math.h"
#include "core/model/entity_data.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace {

using ::imp::gltf::imp_proto::Image;
using ::imp::gltf::imp_proto::Material;
using ::imp::gltf::imp_proto::Primitive;
using ::imp::gltf::imp_proto::Texture;
using MinFilter = schemas::MinFilter;
using MagFilter = schemas::MagFilter;
using WrapMode = schemas::WrapMode;
using TextureInfoFlags = schemas::TextureInfoFlags;
using PathOrIndex = absl::variant<std::string, int>;
using InternalFormat = ::filament::Texture::InternalFormat;

std::string GetTextureNameHelper(const Image& image, int index) {
  if (!image.uri.empty()) {
    return std::string(image.uri);
  } else if (!image.name.empty()) {
    return std::string(image.name);
  } else {
    return absl::StrFormat("%d", index);
  }
}

// Helper method to determine if material should use lighting.
schemas::GenericMaterialLightingModel GetLightingModel(
    const Material& material) {
  if (material.extensions.unlit) {
    return schemas::GenericMaterialLightingModel::Unlit;
  } else {
    return schemas::GenericMaterialLightingModel::Lit;
  }
}

GenericMaterialSpec CreateGenericMaterialSpec(
    const std::string& alpha_mode_string,
    schemas::GenericMaterialLightingModel lighting_model,
    bool is_double_sided) {
  constexpr const char* kMask = "MASK";
  constexpr const char* kBlend = "BLEND";
  constexpr const char* kRefractive = "REFRACTIVE";
  auto dm = (is_double_sided)
                ? schemas::GenericMaterialDoubleSidedMode::DoubleSided
                : schemas::GenericMaterialDoubleSidedMode::SingleSided;
  if (alpha_mode_string == kMask) {
    return GenericMaterialSpec(
        lighting_model, schemas::GenericMaterialBlendMode::Masked, dm,
        schemas::GenericMaterialDepthClearMaterial::Disabled);
  } else if (alpha_mode_string == kBlend) {
    return GenericMaterialSpec(
        lighting_model, schemas::GenericMaterialBlendMode::Transparent, dm,
        schemas::GenericMaterialDepthClearMaterial::Disabled);
  } else if (alpha_mode_string == kRefractive) {
    return GenericMaterialSpec(
        lighting_model, schemas::GenericMaterialBlendMode::Refractive, dm,
        schemas::GenericMaterialDepthClearMaterial::Disabled);
  } else {
    return GenericMaterialSpec(
        lighting_model, schemas::GenericMaterialBlendMode::Opaque, dm,
        schemas::GenericMaterialDepthClearMaterial::Disabled);
  }
}

GenericMaterialSpec GetGenericMaterialSpecFromMaterial(
    const Material& material, bool use_lite_materials) {
  absl::string_view alpha_mode = material.alpha_mode;
  if ((material.extensions.transmission && !use_lite_materials &&
       (alpha_mode.empty() || alpha_mode == "OPAQUE"))) {
    alpha_mode = "REFRACTIVE";
  } else if (alpha_mode.empty()) {
    alpha_mode = "OPAQUE";
  }
  bool is_double_sided = material.double_sided;

  return CreateGenericMaterialSpec(std::string(alpha_mode),
                                   GetLightingModel(material), is_double_sided);
}

filament::math::mat3f ComputeUvMatrix(
    imp::gltf::imp_proto::TextureTransform* xform) {
  float2 offset(0);
  float2 scale(1);
  float rotation(0);

  if (xform) {
    if (xform->offset.size() == 2) {
      offset = float2(xform->offset[0], xform->offset[1]);
    }
    if (xform->scale.size() == 2) {
      scale = float2(xform->scale[0], xform->scale[1]);
    }
    rotation = xform->rotation;
  }
  return MatrixFromUvTransform(offset, rotation, scale);
}

flatbuffers::Offset<schemas::BoundsInfo> CreateBoundsInfo(
    flatbuffers::FlatBufferBuilder& fbb,
    const absl::optional<filament::Box>& bounds) {
  if (bounds) {
    const auto center = bounds->center;
    const auto half_extent = bounds->halfExtent;
    schemas::Box box(flatbuffers::Pack(center), flatbuffers::Pack(half_extent));

    return schemas::CreateBoundsInfo(fbb, &box);
  } else {
    return flatbuffers::Offset<schemas::BoundsInfo>{};
  }
}

filament::TextureSampler::MinFilter ConvertMinFilter(
    imp::gltf::imp_proto::Sampler::MinMagFilter filter) {
  switch (filter) {
    case gltf::imp_proto::Sampler::NEAREST:
      return filament::TextureSampler::MinFilter::NEAREST;
    case gltf::imp_proto::Sampler::LINEAR:
      return filament::TextureSampler::MinFilter::LINEAR;
    case gltf::imp_proto::Sampler::NEAREST_MIPMAP_NEAREST:
      return filament::TextureSampler::MinFilter::NEAREST_MIPMAP_NEAREST;
    case gltf::imp_proto::Sampler::LINEAR_MIPMAP_NEAREST:
      return filament::TextureSampler::MinFilter::LINEAR_MIPMAP_NEAREST;
    case gltf::imp_proto::Sampler::NEAREST_MIPMAP_LINEAR:
      return filament::TextureSampler::MinFilter::NEAREST_MIPMAP_NEAREST;
    default:
    case gltf::imp_proto::Sampler::LINEAR_MIPMAP_LINEAR:
      return filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR;
      break;
  }
}

filament::TextureSampler::MagFilter ConvertMagFilter(
    imp::gltf::imp_proto::Sampler::MinMagFilter filter) {
  switch (filter) {
    case imp::gltf::imp_proto::Sampler::NEAREST:
      return filament::TextureSampler::MagFilter::NEAREST;
    default:
    case imp::gltf::imp_proto::Sampler::LINEAR:
      return filament::TextureSampler::MagFilter::LINEAR;
  }
}

filament::TextureSampler::WrapMode ConvertWrapMode(
    imp::gltf::imp_proto::Sampler::WrapMode wrap_mode) {
  switch (wrap_mode) {
    case imp::gltf::imp_proto::Sampler::CLAMP_TO_EDGE:
      return filament::TextureSampler::WrapMode::CLAMP_TO_EDGE;
    case imp::gltf::imp_proto::Sampler::REPEAT:
    default:
      return filament::TextureSampler::WrapMode::REPEAT;
    case imp::gltf::imp_proto::Sampler::MIRRORED_REPEAT:
      return filament::TextureSampler::WrapMode::MIRRORED_REPEAT;
  }
}

filament::TextureSampler ConvertSampler(
    const gltf::imp_proto::Sampler& sampler) {
  filament::TextureSampler result(
      ConvertMinFilter(sampler.min_filter),
      ConvertMagFilter(sampler.mag_filter), ConvertWrapMode(sampler.wrap_s),
      ConvertWrapMode(sampler.wrap_t),
      ConvertWrapMode(imp::gltf::imp_proto::Sampler::REPEAT));
  result.setCompareMode(filament::TextureSampler::CompareMode::NONE,
                        filament::TextureSampler::CompareFunc::LE);
  result.setAnisotropy(0);
  return result;
}

template <typename TextureInfo>
absl::StatusOr<std::optional<GenericMaterialTextureParameter>>
CreateGenericMaterialTextureParameter(
    loader::details::LoadedModelBuilder& builder,
    const loader::details::provider_gltf::GltfModel& gltf_model,
    const TextureInfo& tex_info) {
  if (!tex_info.index) {
    return std::nullopt;
  }

  // Find the lookup index for the texture image in the loaded model.
  int texture_index = *tex_info.index;
  if (texture_index < 0 || texture_index >= gltf_model.GetTextureCount()) {
    return absl::NotFoundError(absl::StrFormat(
        "Gltf requested an invalid texture index: %d, texture count: %d",
        texture_index, gltf_model.GetTextureCount()));
  }
  const Texture& texture = gltf_model.GetTexture(texture_index);
  MP_ASSIGN_OR_RETURN(
      uint16_t texture_lookup_index,
      loader::details::provider_gltf::GetTextureLookupIndex(texture));

  // Find the texcoord for this texture.
  //
  // If the texcoord is present in the KHR_texture_transform extension it
  // overrides the one set on the texture itself.
  int tex_coord = tex_info.tex_coord;
  if (tex_info.extensions.xform) {
    const std::unique_ptr<gltf::imp_proto::TextureTransform>& xform =
        tex_info.extensions.xform;
    if (xform->tex_coord) {
      tex_coord = xform->tex_coord.value();
    }
  }

  bool uses_uv1 = false;
  // If this texture uses UV1, set the bitflag.
  if (tex_coord == 1) {
    uses_uv1 = true;
  }

  model::TextureId texture_id = builder.GetTexture(texture_lookup_index);
  if (texture_id == model::TextureId{}) {
    return absl::NotFoundError(absl::StrFormat(
        "Gltf requested an invalid texture index: %d for texture %s",
        texture_lookup_index, texture.name));
  }

  // Get the sampler index for the texture.
  gltf::imp_proto::Sampler sampler;
  uint16_t sampler_index = texture.sampler ? *texture.sampler : -1;
  if (sampler_index >= 0 && sampler_index < gltf_model.GetSamplerCount()) {
    sampler = gltf_model.GetSampler(sampler_index);
  }

  return GenericMaterialTextureParameter{
      static_cast<uint16_t>(texture_id), ConvertSampler(sampler),
      ComputeUvMatrix(tex_info.extensions.xform.get()), uses_uv1};
}

}  // namespace

absl::Status CreateGenericMaterialSchemas(
    loader::details::LoadedModelBuilder& builder,
    const loader::details::provider_gltf::GltfModel& model,
    const std::vector<imp::gltf::imp_proto::Primitive>& primitives,
    const loader::details::provider_gltf::GltfPrimitiveVector<
        loader::details::provider_gltf::ProcessedPrimitive>&
        processed_primitives,
    const loader::details::provider_gltf::Gltf2AttributeMask& mask,
    bool use_lite_materials) {
  if (primitives.empty()) {
    return absl::InvalidArgumentError("Mesh contains no geometry.");
  }

  flatbuffers::FlatBufferBuilder& fbb = builder.GetFlatBufferBuilder();

  for (auto& primitive : primitives) {
    size_t primitive_index = &primitive - &primitives.front();
    const loader::details::provider_gltf::ProcessedPrimitive&
        processed_primitive = processed_primitives[primitive_index];
    for (auto material_index : processed_primitive.required_materials) {
      if (builder.GetMaterial(material_index) != model::MaterialId{}) continue;

      // -1 is the Default material. Access the default material by passing
      // absl::nullopt into GetMaterial.
      const gltf::imp_proto::Material& m = model.GetMaterial(
          material_index == -1 ? absl::nullopt
                               : absl::optional<uint32_t>(material_index));

      filament::Box root_bounds;
      if (m.extensions.mask) {
        // Create a depth clear material for the mask.
        GenericMaterialSpec generic_material_spec(
            schemas::GenericMaterialLightingModel::Unlit,
            schemas::GenericMaterialBlendMode::Opaque,
            schemas::GenericMaterialDoubleSidedMode::SingleSided,
            schemas::GenericMaterialDepthClearMaterial::Enabled);
        builder.AddMaterial(
            material_index,
            schemas::CreateMaterialInfo(
                fbb,
                schemas::CreateGenericMaterial(
                    fbb, fbb.CreateString(m.name.data(), m.name.size()),
                    generic_material_spec.ToFlatbuffer(fbb),
                    CreateBoundsInfo(fbb, root_bounds), {}),
                material_index));
        continue;
      }

      GenericMaterialParameters generic_material_parameters;
      generic_material_parameters.base_color.emplace();
      generic_material_parameters.base_color->factor = kOne4;
      if (m.pbr_metallic_roughness.base_color_factor.size() == 4) {
        generic_material_parameters.base_color->factor[0] =
            m.pbr_metallic_roughness.base_color_factor[0];
        generic_material_parameters.base_color->factor[1] =
            m.pbr_metallic_roughness.base_color_factor[1];
        generic_material_parameters.base_color->factor[2] =
            m.pbr_metallic_roughness.base_color_factor[2];
        generic_material_parameters.base_color->factor[3] =
            m.pbr_metallic_roughness.base_color_factor[3];
      }

      generic_material_parameters.metallic_roughness.emplace();
      generic_material_parameters.metallic_roughness->metallic_factor =
          m.pbr_metallic_roughness.metallic_factor.value_or(1.0f);
      generic_material_parameters.metallic_roughness->roughness_factor =
          m.pbr_metallic_roughness.roughness_factor.value_or(1.0f);

      generic_material_parameters.normal.emplace();
      generic_material_parameters.normal->factor =
          m.normal_texture.scale.value_or(1.0f);

      generic_material_parameters.ambient_occlusion.emplace();
      generic_material_parameters.ambient_occlusion->factor =
          m.occlusion_texture.strength.value_or(1.0f);

      generic_material_parameters.emissive.emplace();
      generic_material_parameters.emissive->factor = kZero3;
      if (m.emissive_factor.size() == 3) {
        generic_material_parameters.emissive->factor[0] = m.emissive_factor[0];
        generic_material_parameters.emissive->factor[1] = m.emissive_factor[1];
        generic_material_parameters.emissive->factor[2] = m.emissive_factor[2];
      }

      if (m.extensions.pbr_specular_glossiness) {
        const auto& specular_glossiness = *m.extensions.pbr_specular_glossiness;
        if (specular_glossiness.diffuse_factor.size() == 4) {
          generic_material_parameters.base_color->factor[0] =
              specular_glossiness.diffuse_factor[0];
          generic_material_parameters.base_color->factor[1] =
              specular_glossiness.diffuse_factor[1];
          generic_material_parameters.base_color->factor[2] =
              specular_glossiness.diffuse_factor[2];
          generic_material_parameters.base_color->factor[3] =
              specular_glossiness.diffuse_factor[3];
        }
        float glossiness_factor = specular_glossiness.glossiness_factor
                                      ? *specular_glossiness.glossiness_factor
                                      : 1.0f;
        generic_material_parameters.metallic_roughness->roughness_factor =
            powf(1.0f - glossiness_factor, 2.0f);
      }

      // Base Color Texture.
      auto& base_color_texture =
          m.extensions.pbr_specular_glossiness
              ? m.extensions.pbr_specular_glossiness->diffuse_texture
              : m.pbr_metallic_roughness.base_color_texture;

      MP_ASSIGN_OR_RETURN(generic_material_parameters.base_color->texture,
                       CreateGenericMaterialTextureParameter(
                           builder, model, base_color_texture));

      schemas::GenericMaterialLightingModel lighting_model =
          GetLightingModel(m);
      if (lighting_model == schemas::GenericMaterialLightingModel::Lit) {
        // Metallic Roughness Texture.
        MP_ASSIGN_OR_RETURN(
            generic_material_parameters.metallic_roughness->texture,
            CreateGenericMaterialTextureParameter(
                builder, model,
                m.pbr_metallic_roughness.metallic_roughness_texture));

        // Normal Texture.
        MP_ASSIGN_OR_RETURN(generic_material_parameters.normal->texture,
                         CreateGenericMaterialTextureParameter(
                             builder, model, m.normal_texture));

        // Occlusion Texture.
        MP_ASSIGN_OR_RETURN(generic_material_parameters.ambient_occlusion->texture,
                         CreateGenericMaterialTextureParameter(
                             builder, model, m.occlusion_texture));

        // Emissive Texture.
        MP_ASSIGN_OR_RETURN(generic_material_parameters.emissive->texture,
                         CreateGenericMaterialTextureParameter(
                             builder, model, m.emissive_texture));

        // KHR_materials_clearcoat
        if (lighting_model == schemas::GenericMaterialLightingModel::Lit) {
          if (m.extensions.clearcoat) {
            auto& clearcoat = *m.extensions.clearcoat;

            generic_material_parameters.clearcoat.emplace();

            MP_ASSIGN_OR_RETURN(
                generic_material_parameters.clearcoat->intensity_texture,
                CreateGenericMaterialTextureParameter(
                    builder, model, clearcoat.clearcoat_texture));

            MP_ASSIGN_OR_RETURN(
                generic_material_parameters.clearcoat->normal_texture,
                CreateGenericMaterialTextureParameter(
                    builder, model, clearcoat.clearcoat_normal_texture));

            MP_ASSIGN_OR_RETURN(
                generic_material_parameters.clearcoat->roughness_texture,
                CreateGenericMaterialTextureParameter(
                    builder, model, clearcoat.clearcoat_roughness_texture));

            generic_material_parameters.clearcoat->factor = {
                clearcoat.clearcoat_factor.value_or(0.0f),
                clearcoat.clearcoat_roughness_factor.value_or(0.0f),
                clearcoat.clearcoat_normal_texture.scale.value_or(1.0f)};
          }
        }

        // KHR_materials_sheen
        if (m.extensions.sheen) {
          const std::unique_ptr<gltf::imp_proto::MaterialSheen>& sheen_info =
              m.extensions.sheen;

          generic_material_parameters.sheen.emplace();

          MP_ASSIGN_OR_RETURN(
              generic_material_parameters.sheen->color_texture,
              CreateGenericMaterialTextureParameter(
                  builder, model, sheen_info->sheen_color_texture));

          generic_material_parameters.sheen->roughness_factor =
              sheen_info->sheen_roughness_factor;

          MP_ASSIGN_OR_RETURN(
              generic_material_parameters.sheen->roughness_texture,
              CreateGenericMaterialTextureParameter(
                  builder, model, sheen_info->sheen_roughness_texture));

          if (sheen_info->sheen_color_factor.size() == 3) {
            generic_material_parameters.sheen->color_factor = {
                sheen_info->sheen_color_factor[0],
                sheen_info->sheen_color_factor[1],
                sheen_info->sheen_color_factor[2]};
          }
        }

        // KHR_materials_ior
        if (!use_lite_materials) {
          float ior = kDefaultIndexOfRefraction;
          if (m.extensions.ior) {
            ior = m.extensions.ior->ior.value_or(ior);
          }

          generic_material_parameters.refraction.emplace();
          generic_material_parameters.refraction->index_of_refraction = ior;

          // KHR_materials_transmission
          if (m.extensions.transmission) {
            generic_material_parameters.transmission.emplace();
            // Transmission Texture.
            MP_ASSIGN_OR_RETURN(
                generic_material_parameters.transmission->texture,
                CreateGenericMaterialTextureParameter(
                    builder, model,
                    m.extensions.transmission->transmission_texture));

            generic_material_parameters.transmission->factor =
                m.extensions.transmission->transmission_factor.value_or(0.0f);
          }
        }
      }

      // EXT_mesh_features
      if (!processed_primitive.feature_id_textures.empty()) {
        std::vector<GenericMaterialTextureParameter> feature_ids;
        feature_ids.reserve(processed_primitive.feature_id_textures.size());
        for (const auto& feature_id_texture :
             processed_primitive.feature_id_textures) {
          std::optional<GenericMaterialTextureParameter>
              feature_id_texture_parameter;
          MP_ASSIGN_OR_RETURN(feature_id_texture_parameter,
                           CreateGenericMaterialTextureParameter(
                               builder, model, feature_id_texture));
          if (feature_id_texture_parameter.has_value()) {
            feature_ids.emplace_back(*feature_id_texture_parameter);
          }
        }
        generic_material_parameters.feature_id_textures =
            std::move(feature_ids);
      }

      if (m.alpha_cutoff.has_value()) {
        generic_material_parameters.masking.emplace();
        generic_material_parameters.masking->alpha_cutoff = *m.alpha_cutoff;
      }

      GenericMaterialSpec generic_material_spec =
          GetGenericMaterialSpecFromMaterial(m, use_lite_materials);

      flatbuffers::Offset<schemas::GenericMaterialParameters>
          generic_material_parameters_offset =
              generic_material_parameters.ToFlatbuffer(fbb);

      builder.AddMaterial(
          material_index,
          schemas::CreateMaterialInfo(
              fbb,
              schemas::CreateGenericMaterial(
                  fbb, fbb.CreateString(m.name.data(), m.name.size()),
                  generic_material_spec.ToFlatbuffer(fbb),
                  CreateBoundsInfo(fbb, processed_primitive.root_bounds),
                  generic_material_parameters_offset),
              material_index));
    }
  }
  return absl::OkStatus();
}

}  // namespace imp
