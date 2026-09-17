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

#include "core/split_engine/materials/split_engine_generic_material.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/functional/function_ref.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/common/owned_ptr.h"
#include "core/common/small_source_location.h"
#include "core/common/typed_vector.h"
#include "core/material_library/generic_material_constants.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_param_value.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/model/entity_data.h"
#include "core/render/texture.h"
#include "core/split_engine/flatbuffer_size_calculator.h"
#include "core/split_engine/material_requester.h"  // IWYU pragma: keep
#include "core/split_engine/materials/builtin/builtin_generic_spec_helpers.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/split_engine/transport/request_sender.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/utils/string_map.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "split_engine/schemas/split_engine_schema_version.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

namespace {

// A helper class to convert from Split Engine schema tables to C++ structs.
//
// This is part of a system that lets both Impress glTF loader and Split Engine
// material schemas be converted to the GenericMaterialParameters struct.
//
// Note: These constexprs are used by templated code, which the linter doesn't
// understand.
class SplitEngineGenericMaterialParametersCreator {
 public:
  using Schema = android_xr::schemas::GenericMaterialParameters;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParameters(Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialParameters(
        std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialTextureParameter =
      android_xr::schemas::GenericMaterialTextureParameter;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialTextureParameter(Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialTextureParameter(
        std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialParametersBaseColor =
      android_xr::schemas::GenericMaterialParametersBaseColor;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParametersBaseColor(
      Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialParametersBaseColor(
        std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialParametersMetallicRoughness =
      android_xr::schemas::GenericMaterialParametersMetallicRoughness;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParametersMetallicRoughness(
      Args&&... args) {
    return android_xr::schemas::
        CreateGenericMaterialParametersMetallicRoughness(
            std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialParametersNormal =
      android_xr::schemas::GenericMaterialParametersNormal;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParametersNormal(Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialParametersNormal(
        std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialParametersAmbientOcclusion =
      android_xr::schemas::GenericMaterialParametersAmbientOcclusion;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParametersAmbientOcclusion(
      Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialParametersAmbientOcclusion(
        std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialParametersEmissive =
      android_xr::schemas::GenericMaterialParametersEmissive;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParametersEmissive(
      Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialParametersEmissive(
        std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialParametersClearcoat =
      android_xr::schemas::GenericMaterialParametersClearcoat;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParametersClearcoat(
      Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialParametersClearcoat(
        std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialParametersSheen =
      android_xr::schemas::GenericMaterialParametersSheen;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParametersSheen(Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialParametersSheen(
        std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialParametersTransmission =
      android_xr::schemas::GenericMaterialParametersTransmission;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParametersTransmission(
      Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialParametersTransmission(
        std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialParametersVolume =
      android_xr::schemas::GenericMaterialParametersVolume;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParametersVolume(Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialParametersVolume(
        std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialParametersRefraction =
      android_xr::schemas::GenericMaterialParametersRefraction;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParametersRefraction(
      Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialParametersRefraction(
        std::forward<decltype(args)>(args)...);
  };

  using GenericMaterialParametersMasking =
      android_xr::schemas::GenericMaterialParametersMasking;
  template <typename... Args>
  static constexpr auto CreateGenericMaterialParametersMasking(Args&&... args) {
    return android_xr::schemas::CreateGenericMaterialParametersMasking(
        std::forward<decltype(args)>(args)...);
  };

  using Bool = android_xr::schemas::Bool;
  using Float = android_xr::schemas::Float;
  using Float2 = android_xr::schemas::Float2;
  using Float3 = android_xr::schemas::Float3;
  using Float4 = android_xr::schemas::Float4;
  using Mat3f = android_xr::schemas::Mat3f;

  using TextureSamplerCreator = SplitEngineTextureSamplerCreator;
};

std::optional<GenericMaterialTextureParameter>* GetTextureField(
    GenericMaterialParameters& params, absl::string_view channel_name,
    bool create_if_null) {
  if (channel_name == kBaseColorIndex) {
    if (!params.base_color) {
      if (!create_if_null) return nullptr;
      params.base_color.emplace();
    }
    return &params.base_color->texture;
  } else if (channel_name == kMetallicRoughnessIndex) {
    if (!params.metallic_roughness) {
      if (!create_if_null) return nullptr;
      params.metallic_roughness.emplace();
    }
    return &params.metallic_roughness->texture;
  } else if (channel_name == kNormalIndex) {
    if (!params.normal) {
      if (!create_if_null) return nullptr;
      params.normal.emplace();
    }
    return &params.normal->texture;
  } else if (channel_name == kAoIndex) {
    if (!params.ambient_occlusion) {
      if (!create_if_null) return nullptr;
      params.ambient_occlusion.emplace();
    }
    return &params.ambient_occlusion->texture;
  } else if (channel_name == kEmissiveIndex) {
    if (!params.emissive) {
      if (!create_if_null) return nullptr;
      params.emissive.emplace();
    }
    return &params.emissive->texture;
  } else if (channel_name == kClearcoatIndex) {
    if (!params.clearcoat) {
      if (!create_if_null) return nullptr;
      params.clearcoat.emplace();
    }
    return &params.clearcoat->intensity_texture;
  } else if (channel_name == kClearcoatNormalIndex) {
    if (!params.clearcoat) {
      if (!create_if_null) return nullptr;
      params.clearcoat.emplace();
    }
    return &params.clearcoat->normal_texture;
  } else if (channel_name == kClearcoatRoughnessIndex) {
    if (!params.clearcoat) {
      if (!create_if_null) return nullptr;
      params.clearcoat.emplace();
    }
    return &params.clearcoat->roughness_texture;
  } else if (channel_name == kSheenColorIndex) {
    if (!params.sheen) {
      if (!create_if_null) return nullptr;
      params.sheen.emplace();
    }
    return &params.sheen->color_texture;
  } else if (channel_name == kSheenRoughnessIndex) {
    if (!params.sheen) {
      if (!create_if_null) return nullptr;
      params.sheen.emplace();
    }
    return &params.sheen->roughness_texture;
  } else if (channel_name == kTransmissionIndex) {
    if (!params.transmission) {
      if (!create_if_null) return nullptr;
      params.transmission.emplace();
    }
    return &params.transmission->texture;
  } else if (channel_name == kThicknessIndex) {
    if (!params.volume) {
      if (!create_if_null) return nullptr;
      params.volume.emplace();
    }
    return &params.volume->thickness_texture;
  }
  return nullptr;
}

}  // namespace

Future<std::unique_ptr<SplitEngineGenericMaterial>>
SplitEngineGenericMaterial::Create(BaseView& view,
                                   const GenericMaterialSpec& spec) {
  if (spec.GetDepthClearMaterial() ==
      schemas::GenericMaterialDepthClearMaterial::Enabled) {
    return Future<std::unique_ptr<SplitEngineGenericMaterial>>(
        absl::UnimplementedError(
            "GenericMaterialSpec::schema::GenericMaterialDepthClearMaterial is "
            "not supported in Split Engine."));
  }

  absl::StatusOr<RequestSender::RequestBuilder> builder =
      SplitEngineBuiltinMaterial::CreateFlatBufferBuilder(
          view, FlatbufferSizeCalculator()
                    .AddRequestBuiltInGenericMaterial()
                    .Finish()
                    .AddScratchSpace()
                    .ComputeSize());
  MP_RETURN_IF_ERROR(builder.status());

  RequestSender::RequestBuilder fbb = *std::move(builder);
  absl::StatusOr<flatbuffers::Offset<android_xr::schemas::GenericMaterialSpec>>
      spec_offset = Pack(*fbb, spec);
  if (!spec_offset.ok()) {
    return Future<std::unique_ptr<SplitEngineGenericMaterial>>(
        spec_offset.status());
  }

  return SplitEngineBuiltinMaterial::CreatePlaceholderMaterial(view).Then(
      [&view, fbb = std::move(fbb),
       spec_offset](OwnedMaterialPtr placeholder_material) mutable {
        flatbuffers::Offset<android_xr::schemas::BuiltInMaterialRequest>
            built_in_material_request =
                android_xr::schemas::CreateBuiltInMaterialRequest(
                    *fbb,
                    SplitEngineSerializer::GetId(
                        placeholder_material->GetFilamentMaterialInstance()),
                    android_xr::schemas::BuiltInMaterialSpec::
                        GenericMaterialSpec,
                    spec_offset->Union());
        return view.GetSplitEngineSerializer()
            ->GetMaterialRequester()
            .RequestBuiltInMaterialInstance(std::move(fbb),
                                            built_in_material_request)
            .Then([&view, placeholder_material =
                              std::move(placeholder_material)]() mutable {
              return absl::WrapUnique(new SplitEngineGenericMaterial(
                  view, std::move(placeholder_material)));
            });
      });
}

SplitEngineGenericMaterial::SplitEngineGenericMaterial(
    BaseView& view, OwnedMaterialPtr placeholder_material)
    : SplitEngineBuiltinMaterial(
          view,
          // Note: this is the union enum type of the new schema
          // but the old schema is still used for serialization.
          // TODO: (broken link) - remove this comment.
          android_xr::schemas::BuiltInMaterialParameters::
              GenericMaterialParameters,
          std::move(placeholder_material)),
      placeholder_texture_(
          view.GetTextureFactory().BorrowPlaceholderTexture()) {
  SetName("SplitEngineGenericMaterial");
}

SplitEngineGenericMaterial::~SplitEngineGenericMaterial() { Cleanup(); }

const std::string& SplitEngineGenericMaterial::GetName() const { return name_; }

void SplitEngineGenericMaterial::SetName(absl::string_view name) {
  name_ = name;
}

const filament::MaterialInstance*
SplitEngineGenericMaterial::GetFilamentMaterialInstance() const {
  return GetMaterialInternal(SmallSourceLocation::Current())
      ->GetFilamentMaterialInstance();
}

filament::MaterialInstance*
SplitEngineGenericMaterial::GetFilamentMaterialInstance() {
  return GetMaterialInternal(SmallSourceLocation::Current())
      ->GetFilamentMaterialInstance();
}

TextureAndSampler SplitEngineGenericMaterial::GetBaseColorTexture() const {
  if (!generic_material_parameters_.base_color.has_value() ||
      !generic_material_parameters_.base_color->texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(
      *generic_material_parameters_.base_color->texture);
}

void SplitEngineGenericMaterial::SetBaseColorFactor(const float4& factor) {
  if (!generic_material_parameters_.base_color.has_value()) {
    generic_material_parameters_.base_color.emplace();
  }
  generic_material_parameters_.base_color->factor = factor;
  MarkParametersDirty();
}

float4 SplitEngineGenericMaterial::GetBaseColorFactor() const {
  if (!generic_material_parameters_.base_color.has_value()) {
    return kDefaultBaseColorFactor;
  }

  return generic_material_parameters_.base_color->factor;
}

absl::Status SplitEngineGenericMaterial::SetBaseColorUvTransform(
    const mat3f& uv_transform) {
  if (!generic_material_parameters_.base_color.has_value() ||
      !generic_material_parameters_.base_color->texture) {
    return absl::UnavailableError("Base color texture is not assigned.");
  }
  generic_material_parameters_.base_color->texture->uv_transform = uv_transform;
  MarkParametersDirty();
  return absl::OkStatus();
}

TextureAndSampler SplitEngineGenericMaterial::GetMetallicRoughnessTexture()
    const {
  if (!generic_material_parameters_.metallic_roughness.has_value() ||
      !generic_material_parameters_.metallic_roughness->texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(
      *generic_material_parameters_.metallic_roughness->texture);
}

absl::Status SplitEngineGenericMaterial::SetMetallicRoughnessUvTransform(
    const mat3f& uv_transform) {
  if (!generic_material_parameters_.metallic_roughness.has_value() ||
      !generic_material_parameters_.metallic_roughness->texture) {
    return absl::UnavailableError(
        "Metallic Roughness texture is not assigned.");
  }
  generic_material_parameters_.metallic_roughness->texture->uv_transform =
      uv_transform;
  MarkParametersDirty();
  return absl::OkStatus();
}

void SplitEngineGenericMaterial::SetMetallicFactor(float factor) {
  if (!generic_material_parameters_.metallic_roughness.has_value()) {
    generic_material_parameters_.metallic_roughness.emplace();
  }
  generic_material_parameters_.metallic_roughness->metallic_factor = factor;
  MarkParametersDirty();
}

float SplitEngineGenericMaterial::GetMetallicFactor() const {
  if (!generic_material_parameters_.metallic_roughness.has_value()) {
    return kDefaultMetallicFactor;
  }
  return generic_material_parameters_.metallic_roughness->metallic_factor;
}

void SplitEngineGenericMaterial::SetRoughnessFactor(float factor) {
  if (!generic_material_parameters_.metallic_roughness.has_value()) {
    generic_material_parameters_.metallic_roughness.emplace();
  }
  generic_material_parameters_.metallic_roughness->roughness_factor = factor;
  MarkParametersDirty();
}

float SplitEngineGenericMaterial::GetRoughnessFactor() const {
  if (!generic_material_parameters_.metallic_roughness.has_value()) {
    return kDefaultRoughnessFactor;
  }
  return generic_material_parameters_.metallic_roughness->roughness_factor;
}

TextureAndSampler SplitEngineGenericMaterial::GetNormalTexture() const {
  if (!generic_material_parameters_.normal.has_value() ||
      !generic_material_parameters_.normal->texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(*generic_material_parameters_.normal->texture);
}

absl::Status SplitEngineGenericMaterial::SetNormalUvTransform(
    const mat3f& uv_transform) {
  if (!generic_material_parameters_.normal.has_value() ||
      !generic_material_parameters_.normal->texture) {
    return absl::UnavailableError("Normal texture is not assigned.");
  }
  generic_material_parameters_.normal->texture->uv_transform = uv_transform;
  MarkParametersDirty();
  return absl::OkStatus();
}

void SplitEngineGenericMaterial::SetNormalScale(float scale) {
  if (!generic_material_parameters_.normal.has_value()) {
    generic_material_parameters_.normal.emplace();
  }
  generic_material_parameters_.normal->factor = scale;
  MarkParametersDirty();
}

float SplitEngineGenericMaterial::GetNormalScale() const {
  if (!generic_material_parameters_.normal.has_value()) {
    return kDefaultNormalFactor;
  }
  return generic_material_parameters_.normal->factor;
}

TextureAndSampler SplitEngineGenericMaterial::GetAmbientOcclusionTexture()
    const {
  if (!generic_material_parameters_.ambient_occlusion.has_value() ||
      !generic_material_parameters_.ambient_occlusion->texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(
      *generic_material_parameters_.ambient_occlusion->texture);
}

absl::Status SplitEngineGenericMaterial::SetAmbientOcclusionUvTransform(
    const mat3f& uv_transform) {
  if (!generic_material_parameters_.ambient_occlusion.has_value() ||
      !generic_material_parameters_.ambient_occlusion->texture) {
    return absl::UnavailableError("Ambient Occlusion texture is not assigned.");
  }
  generic_material_parameters_.ambient_occlusion->texture->uv_transform =
      uv_transform;
  MarkParametersDirty();
  return absl::OkStatus();
}

void SplitEngineGenericMaterial::SetAmbientOcclusionStrength(float strength) {
  if (!generic_material_parameters_.ambient_occlusion.has_value()) {
    generic_material_parameters_.ambient_occlusion.emplace();
  }
  generic_material_parameters_.ambient_occlusion->factor = strength;
  MarkParametersDirty();
}

float SplitEngineGenericMaterial::GetAmbientOcclusionStrength() const {
  if (!generic_material_parameters_.ambient_occlusion.has_value()) {
    return kDefaultAmbientOcclusionFactor;
  }
  return generic_material_parameters_.ambient_occlusion->factor;
}

TextureAndSampler SplitEngineGenericMaterial::GetEmissiveTexture() const {
  if (!generic_material_parameters_.emissive.has_value() ||
      !generic_material_parameters_.emissive->texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(*generic_material_parameters_.emissive->texture);
}

absl::Status SplitEngineGenericMaterial::SetEmissiveUvTransform(
    const mat3f& uv_transform) {
  if (!generic_material_parameters_.emissive.has_value() ||
      !generic_material_parameters_.emissive->texture) {
    return absl::UnavailableError("Emissive texture is not assigned.");
  }
  generic_material_parameters_.emissive->texture->uv_transform = uv_transform;
  MarkParametersDirty();
  return absl::OkStatus();
}

void SplitEngineGenericMaterial::SetEmissiveFactor(const float3& factor) {
  if (!generic_material_parameters_.emissive.has_value()) {
    generic_material_parameters_.emissive.emplace();
  }
  generic_material_parameters_.emissive->factor = factor;
  MarkParametersDirty();
}

float3 SplitEngineGenericMaterial::GetEmissiveFactor() const {
  if (!generic_material_parameters_.emissive.has_value()) {
    return kDefaultEmissiveFactor;
  }
  return generic_material_parameters_.emissive->factor;
}

TextureAndSampler SplitEngineGenericMaterial::GetClearcoatTexture() const {
  if (!generic_material_parameters_.clearcoat.has_value() ||
      !generic_material_parameters_.clearcoat->intensity_texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(
      *generic_material_parameters_.clearcoat->intensity_texture);
}

TextureAndSampler SplitEngineGenericMaterial::GetClearcoatRoughnessTexture()
    const {
  if (!generic_material_parameters_.clearcoat.has_value() ||
      !generic_material_parameters_.clearcoat->roughness_texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(
      *generic_material_parameters_.clearcoat->roughness_texture);
}

TextureAndSampler SplitEngineGenericMaterial::GetClearcoatNormalTexture()
    const {
  if (!generic_material_parameters_.clearcoat.has_value() ||
      !generic_material_parameters_.clearcoat->normal_texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(
      *generic_material_parameters_.clearcoat->normal_texture);
}

void SplitEngineGenericMaterial::SetClearcoatFactors(const float3& factor) {
  if (!generic_material_parameters_.clearcoat.has_value()) {
    generic_material_parameters_.clearcoat.emplace();
  }
  generic_material_parameters_.clearcoat->factor = factor;
  MarkParametersDirty();
}

TextureAndSampler SplitEngineGenericMaterial::GetSheenColorTexture() const {
  if (!generic_material_parameters_.sheen.has_value() ||
      !generic_material_parameters_.sheen->color_texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(
      *generic_material_parameters_.sheen->color_texture);
}

void SplitEngineGenericMaterial::SetSheenColorFactor(const float3& factor) {
  if (!generic_material_parameters_.sheen.has_value()) {
    generic_material_parameters_.sheen.emplace();
  }
  generic_material_parameters_.sheen->color_factor = factor;
  MarkParametersDirty();
}

TextureAndSampler SplitEngineGenericMaterial::GetSheenRoughnessTexture() const {
  if (!generic_material_parameters_.sheen.has_value() ||
      !generic_material_parameters_.sheen->roughness_texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(
      *generic_material_parameters_.sheen->roughness_texture);
}

void SplitEngineGenericMaterial::SetSheenRoughnessFactor(float factor) {
  if (!generic_material_parameters_.sheen.has_value()) {
    generic_material_parameters_.sheen.emplace();
  }
  generic_material_parameters_.sheen->roughness_factor = factor;
  MarkParametersDirty();
}

void SplitEngineGenericMaterial::SetIndexOfRefraction(
    float index_of_refraction) {
  if (!generic_material_parameters_.refraction.has_value()) {
    generic_material_parameters_.refraction.emplace();
  }
  generic_material_parameters_.refraction->index_of_refraction =
      index_of_refraction;
  MarkParametersDirty();
}

TextureAndSampler SplitEngineGenericMaterial::GetThicknessTexture() const {
  if (!generic_material_parameters_.volume.has_value()) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(
      *generic_material_parameters_.volume->thickness_texture);
}

void SplitEngineGenericMaterial::SetThicknessFactor(float factor) {
  if (!generic_material_parameters_.volume.has_value()) {
    generic_material_parameters_.volume.emplace();
  }
  generic_material_parameters_.volume->thickness_factor = factor;
  MarkParametersDirty();
}

void SplitEngineGenericMaterial::SetAttenuationDistance(float distance) {
  if (!generic_material_parameters_.volume.has_value()) {
    generic_material_parameters_.volume.emplace();
  }
  generic_material_parameters_.volume->attenuation_distance = distance;
  MarkParametersDirty();
}

void SplitEngineGenericMaterial::SetAttenuationColor(const float3& color) {
  if (!generic_material_parameters_.volume.has_value()) {
    generic_material_parameters_.volume.emplace();
  }
  generic_material_parameters_.volume->attenuation_color = color;
  MarkParametersDirty();
}

TextureAndSampler SplitEngineGenericMaterial::GetTransmissionTexture() const {
  if (!generic_material_parameters_.transmission.has_value() ||
      !generic_material_parameters_.transmission->texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(
      *generic_material_parameters_.transmission->texture);
}

absl::Status SplitEngineGenericMaterial::SetTransmissionUvTransform(
    const mat3f& uv_transform) {
  if (!generic_material_parameters_.transmission.has_value() ||
      !generic_material_parameters_.transmission->texture) {
    return absl::UnavailableError("Transmission texture is not assigned.");
  }
  generic_material_parameters_.transmission->texture->uv_transform =
      uv_transform;
  MarkParametersDirty();
  return absl::OkStatus();
}

void SplitEngineGenericMaterial::SetTransmissionFactor(float factor) {
  if (!generic_material_parameters_.transmission.has_value()) {
    generic_material_parameters_.transmission.emplace();
  }
  generic_material_parameters_.transmission->factor = factor;
  MarkParametersDirty();
}

void SplitEngineGenericMaterial::SetAlphaCutoff(float alpha_cutoff) {
  if (!generic_material_parameters_.masking.has_value()) {
    generic_material_parameters_.masking.emplace();
  }
  generic_material_parameters_.masking->alpha_cutoff = alpha_cutoff;
  MarkParametersDirty();
}

float SplitEngineGenericMaterial::GetAlphaCutoff() const {
  if (!generic_material_parameters_.masking.has_value()) {
    return kDefaultAlphaCutoff;
  }

  return generic_material_parameters_.masking->alpha_cutoff;
}

std::optional<TextureAndSampler>
SplitEngineGenericMaterial::GetFeatureIdTexture(int index) const {
  if (!generic_material_parameters_.feature_id_textures.has_value() ||
      index >= generic_material_parameters_.feature_id_textures->size() ||
      index < 0) {
    return std::nullopt;
  }
  return GetTextureAndSampler(
      generic_material_parameters_.feature_id_textures.value()[index]);
}

GenericMaterialPtr SplitEngineGenericMaterial::Duplicate() const {
  // TODO: (broken link) - Make it possible to duplicate imp::Material to avoid
  // losing owned/borrowed texture tracking.
  if (HoldsOwnedTextures()) {
    IMP_LOG(imp::FATAL) << "Cannot duplicate material with owned textures.";
  }

  if (AreParametersDirty()) {
    // Update the parameters before duplicating the material, otherwise the
    // ordering of the parameter update and duplication commands on the renderer
    // side will be incorrect and the duplicate will not contain the update.
    UpdateParameters();
    MarkParametersDirty(false);
  }

  const filament::MaterialInstance* material_instance =
      GetFilamentMaterialInstance();
  filament::MaterialInstance* duplicate_instance =
      filament::MaterialInstance::duplicate(material_instance);
  view_.GetSplitEngineSerializer()->DuplicateMaterialInstance(
      material_instance, duplicate_instance);
  auto duplicate = absl::WrapUnique(new SplitEngineGenericMaterial(
      view_, view_.GetMaterialFactory().WrapMaterial(duplicate_instance)));
  duplicate->generic_material_parameters_ = generic_material_parameters_;
  return duplicate;
}

absl::Status SplitEngineGenericMaterial::AssignTexturesAndParams(
    const GenericMaterialParameters& generic_material_parameters,
    const TextureBorrower& texture_borrower) {
  // Rewrite the texture references to be split engine IDs.
  generic_material_parameters_ =
      RewriteTextureIds(generic_material_parameters, texture_borrower);

  MarkParametersDirty();

  return absl::OkStatus();
}

// TODO: (broken link) - Remove these methods once MaterialConfig is removed.
std::vector<model::MaterialParameter>
SplitEngineGenericMaterial::GetParameters() const {
  return {};
}
TypedVector<model::MaterialTexture> SplitEngineGenericMaterial::GetTextures()
    const {
  return {};
}
StringMap<int> SplitEngineGenericMaterial::GetSamplerIndexLookup() const {
  return {};
}

absl::string_view SplitEngineGenericMaterial::GetParameterTransformName(
    absl::string_view sampler_name) const {
  // We don't know the mapping between sampler names and transform names
  // because the material is held on the SplitEngine renderer side.
  return {};
}

Material::HeldTextureType SplitEngineGenericMaterial::GetAssignedTextureType(
    absl::string_view parameter_name) {
  return HeldTextureType::kBorrowedPointer;
}

imp::StringMap<const filament::Texture*>
SplitEngineGenericMaterial::GetUnownedFilamentTextures() const {
  return {};
}

bool SplitEngineGenericMaterial::HoldsOwnedTextures() const {
  for (auto& [_, texture] : textures_) {
    if (texture.IsOwned()) {
      return true;
    }
  }
  return false;
}

void SplitEngineGenericMaterial::ForEachTexture(
    absl::FunctionRef<void(BorrowedTexturePtr)> fn, SmallSourceLocation loc) {
  for (auto& [_, texture] : textures_) {
    if (texture) {
      fn(texture.Borrow());
    }
  }
}

absl::Status SplitEngineGenericMaterial::SetTextureForChannel(
    absl::string_view channel_name, OwnedOrBorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  BorrowedTexturePtr borrowed_texture = texture.Borrow();
  if (!borrowed_texture) {
    std::optional<GenericMaterialTextureParameter>* field =
        GetTextureField(generic_material_parameters_, channel_name,
                        /*create_if_null=*/false);
    if (field) {
      *field = std::nullopt;
    }
    textures_.erase(channel_name);
    MarkParametersDirty();
    return absl::OkStatus();
  }

  std::optional<GenericMaterialTextureParameter>* field =
      GetTextureField(generic_material_parameters_, channel_name,
                      /*create_if_null=*/true);
  if (!field) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Unsupported texture channel: %s", channel_name));
  }

  TextureId texture_id =
      SplitEngineSerializer::GetId(borrowed_texture->GetTexture());

  GenericMaterialTextureParameter tex_param;
  tex_param.texture_id = texture_id;
  tex_param.sampler =
      sampler_override ? *sampler_override : borrowed_texture->GetSampler();
  // Assign default values.
  tex_param.uv_transform = kIdentityMat3f;
  tex_param.uses_uv1 = false;

  if (*field) {
    tex_param.uv_transform = (*field)->uv_transform;
    tex_param.uses_uv1 = (*field)->uses_uv1;
  }

  *field = tex_param;
  textures_[channel_name.data()] = std::move(texture);
  MarkParametersDirty();
  return absl::OkStatus();
}

BorrowedMaterialPtr SplitEngineGenericMaterial::GetMaterialInternal(
    SmallSourceLocation loc) const {
  return SplitEngineBuiltinMaterial::GetMaterial(loc);
}

flatbuffers::Offset<void> SplitEngineGenericMaterial::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    BuiltInTextureParameterCreator& texture_parameter_creator) const {
  return generic_material_parameters_
      .ToFlatbufferT<SplitEngineGenericMaterialParametersCreator>(fbb)
      .Union();
}

TextureAndSampler SplitEngineGenericMaterial::GetTextureAndSampler(
    const GenericMaterialTextureParameter& texture) const {
  // Note: it is safe to cast this to a filament::Texture* because the ids
  // were processed by RewriteTextureIds.
  return {reinterpret_cast<const filament::Texture*>(texture.texture_id),
          texture.sampler, texture.uv_transform};
}

TextureAndSampler SplitEngineGenericMaterial::GetPlaceholderTextureAndSampler()
    const {
  // TODO: (broken link) - Use placeholder_texture_.Borrow().
  return {placeholder_texture_->GetTexture(), filament::TextureSampler()};
}

void SplitEngineGenericMaterial::RewriteTextureId(
    absl::string_view parameter_name,
    std::optional<GenericMaterialTextureParameter>& texture_parameter,
    const TextureBorrower& texture_borrower) {
  if (!texture_parameter) return;

  BorrowedTexturePtr borrowed = texture_borrower(
      texture_parameter->texture_id, SmallSourceLocation::Current());
  texture_parameter->texture_id =
      SplitEngineSerializer::GetId(borrowed->GetTexture());
  textures_[parameter_name] = std::move(borrowed);
}

GenericMaterialParameters SplitEngineGenericMaterial::RewriteTextureIds(
    const GenericMaterialParameters& parameters,
    const TextureBorrower& texture_borrower) {
  GenericMaterialParameters remapped = parameters;
  if (parameters.base_color) {
    RewriteTextureId(kBaseColorIndex, remapped.base_color->texture,
                     texture_borrower);
  }
  if (parameters.metallic_roughness) {
    RewriteTextureId(kMetallicRoughnessIndex,
                     remapped.metallic_roughness->texture, texture_borrower);
  }
  if (parameters.normal) {
    RewriteTextureId(kNormalIndex, remapped.normal->texture, texture_borrower);
  }
  if (parameters.ambient_occlusion) {
    RewriteTextureId(kAoIndex, remapped.ambient_occlusion->texture,
                     texture_borrower);
  }
  if (parameters.emissive) {
    RewriteTextureId(kEmissiveIndex, remapped.emissive->texture,
                     texture_borrower);
  }
  if (parameters.clearcoat) {
    RewriteTextureId(kClearcoatIndex, remapped.clearcoat->intensity_texture,
                     texture_borrower);
    RewriteTextureId(kClearcoatNormalIndex, remapped.clearcoat->normal_texture,
                     texture_borrower);
    RewriteTextureId(kClearcoatRoughnessIndex,
                     remapped.clearcoat->roughness_texture, texture_borrower);
  }
  if (parameters.sheen) {
    RewriteTextureId(kSheenColorIndex, remapped.sheen->color_texture,
                     texture_borrower);
    RewriteTextureId(kSheenRoughnessIndex, remapped.sheen->roughness_texture,
                     texture_borrower);
  }
  if (parameters.transmission) {
    RewriteTextureId(kTransmissionIndex, remapped.transmission->texture,
                     texture_borrower);
  }
  if (parameters.volume) {
    RewriteTextureId(kThicknessIndex, remapped.volume->thickness_texture,
                     texture_borrower);
  }

  if (parameters.feature_id_textures) {
    int api_level = view_.GetSplitEngineSerializer()->GetApiLevel();
    if (api_level >= 4) {
      if (!remapped.feature_id_textures) {
        remapped.feature_id_textures.emplace();
      }
      if (remapped.feature_id_textures->size() !=
          parameters.feature_id_textures->size()) {
        remapped.feature_id_textures->reserve(kFeatureIdTextureNames.size());
      }
      for (int i = 0; i < parameters.feature_id_textures->size(); ++i) {
        if (i > kFeatureIdTextureNames.size()) break;
        std::optional<GenericMaterialTextureParameter> temp_remapped_texture =
            (*remapped.feature_id_textures)[i];
        absl::string_view feature_id_index = kFeatureIdTextureNames[i];
        RewriteTextureId(feature_id_index, temp_remapped_texture,
                         texture_borrower);
        (*remapped.feature_id_textures)[i] = *temp_remapped_texture;
      }
    } else {
      IMP_LOG(imp::ERROR) << "EXT_mesh_features is not supported over Split Engine - "
                    "requires api level 4+, current api level: "
                 << api_level;
      remapped.feature_id_textures.reset();
    }
  }
  return remapped;
}

}  // namespace imp::split_engine
