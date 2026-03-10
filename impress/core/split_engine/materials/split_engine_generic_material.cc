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
#include <utility>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/functional/function_ref.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
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
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/materials/builtin/builtin_generic_spec_helpers.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/utils/string_map.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "split_engine/schemas/split_engine_schema_version.h"

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
  static constexpr auto CreateGenericMaterialParameters =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParameters;

  using GenericMaterialTextureParameter =
      android_xr::schemas::GenericMaterialTextureParameter;
  static constexpr auto CreateGenericMaterialTextureParameter =  // NOLINT
      android_xr::schemas::CreateGenericMaterialTextureParameter;

  using GenericMaterialParametersBaseColor =
      android_xr::schemas::GenericMaterialParametersBaseColor;
  static constexpr auto CreateGenericMaterialParametersBaseColor =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParametersBaseColor;

  using GenericMaterialParametersMetallicRoughness =
      android_xr::schemas::GenericMaterialParametersMetallicRoughness;
  static constexpr auto
      CreateGenericMaterialParametersMetallicRoughness =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParametersMetallicRoughness;

  using GenericMaterialParametersNormal =
      android_xr::schemas::GenericMaterialParametersNormal;
  static constexpr auto CreateGenericMaterialParametersNormal =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParametersNormal;

  using GenericMaterialParametersAmbientOcclusion =
      android_xr::schemas::GenericMaterialParametersAmbientOcclusion;
  static constexpr auto
      CreateGenericMaterialParametersAmbientOcclusion =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParametersAmbientOcclusion;

  using GenericMaterialParametersEmissive =
      android_xr::schemas::GenericMaterialParametersEmissive;
  static constexpr auto CreateGenericMaterialParametersEmissive =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParametersEmissive;

  using GenericMaterialParametersClearcoat =
      android_xr::schemas::GenericMaterialParametersClearcoat;
  static constexpr auto CreateGenericMaterialParametersClearcoat =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParametersClearcoat;

  using GenericMaterialParametersSheen =
      android_xr::schemas::GenericMaterialParametersSheen;
  static constexpr auto CreateGenericMaterialParametersSheen =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParametersSheen;

  using GenericMaterialParametersTransmission =
      android_xr::schemas::GenericMaterialParametersTransmission;
  static constexpr auto CreateGenericMaterialParametersTransmission =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParametersTransmission;

  using GenericMaterialParametersVolume =
      android_xr::schemas::GenericMaterialParametersVolume;
  static constexpr auto CreateGenericMaterialParametersVolume =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParametersVolume;

  using GenericMaterialParametersRefraction =
      android_xr::schemas::GenericMaterialParametersRefraction;
  static constexpr auto CreateGenericMaterialParametersRefraction =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParametersRefraction;

  using GenericMaterialParametersMasking =
      android_xr::schemas::GenericMaterialParametersMasking;
  static constexpr auto CreateGenericMaterialParametersMasking =  // NOLINT
      android_xr::schemas::CreateGenericMaterialParametersMasking;

  using Bool = android_xr::schemas::Bool;
  using Float = android_xr::schemas::Float;
  using Float2 = android_xr::schemas::Float2;
  using Float3 = android_xr::schemas::Float3;
  using Float4 = android_xr::schemas::Float4;
  using Mat3f = android_xr::schemas::Mat3f;

  using TextureSamplerCreator = SplitEngineTextureSamplerCreator;
};

void LogUnsupportedParameter(absl::string_view name) {
  IMP_LOG(imp::ERROR) << "SetParameter: Unsupported parameter: " << name;
}

void LogParameterNotDirectlyAssignable(absl::string_view name) {
  IMP_LOG(imp::ERROR) << "SetParameter: Parameter is not directly assignable: " << name;
}

void LogTextureParameterNotDirectlyAssignable(absl::string_view name) {
  IMP_LOG(imp::ERROR) << "SetParameter: Texture parameters are not directly assignable: "
             << name << " Use AssignTexture instead.";
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
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  absl::StatusOr<flatbuffers::Offset<android_xr::schemas::GenericMaterialSpec>>
      spec_offset = Pack(*fbb, spec);
  if (!spec_offset.ok()) {
    return Future<std::unique_ptr<SplitEngineGenericMaterial>>(
        spec_offset.status());
  }

  return SplitEngineBuiltinMaterial::CreatePlaceholderMaterial(view).Then(
      [&view, fbb = std::move(fbb),
       spec_offset](OwnedMaterialPtr placeholder_material) {
        flatbuffers::Offset<android_xr::schemas::BuiltInMaterialRequest>
            built_in_material_request =
                android_xr::schemas::CreateBuiltInMaterialRequest(
                    *fbb,
                    SplitEngineSerializer::GetId(
                        placeholder_material->GetFilamentMaterialInstance()),
                    android_xr::schemas::BuiltInMaterialSpec::
                        GenericMaterialSpec,
                    spec_offset->Union());
        return SendRequest<android_xr::schemas::BuiltInMaterialRequest,
                           absl::Status>(
                   view.GetSplitEngineSerializer()->GetBridge(), *fbb,
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
      view_(view),
      placeholder_texture_(
          view.GetTextureFactory().BorrowPlaceholderTexture()) {
  SetName("SplitEngineGenericMaterial");
}

SplitEngineGenericMaterial::~SplitEngineGenericMaterial() { Cleanup(); }

GenericMaterialPtr SplitEngineGenericMaterial::Duplicate() const {
  if (AreParametersDirty()) {
    // Update the parameters before duplicating the material, otherwise the
    // ordering of the parameter update and duplication commands on the renderer
    // side will be incorrect and the duplicate will not contain the update.
    UpdateParameters();
    MarkParametersDirty(false);
  }

  filament::MaterialInstance* material_instance =
      SplitEngineBuiltinMaterial::GetMaterial()->GetFilamentMaterialInstance();
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

flatbuffers::Offset<void> SplitEngineGenericMaterial::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    BuiltInTextureParameterCreator& texture_parameter_creator) const {
  return generic_material_parameters_
      .ToFlatbufferT<SplitEngineGenericMaterialParametersCreator>(fbb)
      .Union();
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

BorrowedMaterialPtr SplitEngineGenericMaterial::GetMaterialInternal(
    SmallSourceLocation loc) const {
  return SplitEngineBuiltinMaterial::GetMaterial(loc);
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
  borrowed_textures_[parameter_name] = std::move(borrowed);
}

// Rewrites the texture IDs from indexes into the loaded model texture listing
// to IDs that can be used to look up textures on the remote renderer.
//
// This is necessary because the glTF doesn't exist on the renderer side.
// Instead, the textures are serialized separately and the IDs are rewritten
// here to reference the texture pointers. A new TextureBorrower is created by
// SplitEngineRenderer that maps these IDs back to concrete textures on the
// backend renderer side.
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
  }
  if (parameters.transmission) {
    RewriteTextureId(kTransmissionIndex, remapped.transmission->texture,
                     texture_borrower);
  }

  if (parameters.feature_id_textures) {
    if (view_.GetSplitEngineSerializer()->GetApiLevel() ==
        android_xr::kSplitEngineExperimentalApiLevel) {
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
      IMP_LOG(imp::ERROR) << "EXT_mesh_features is not supported in this build.";
      remapped.feature_id_textures.reset();
    }
  }
  return remapped;
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

TextureAndSampler SplitEngineGenericMaterial::GetClearcoatNormalTexture()
    const {
  if (!generic_material_parameters_.clearcoat.has_value() ||
      !generic_material_parameters_.clearcoat->normal_texture) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(
      *generic_material_parameters_.clearcoat->normal_texture);
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

TextureAndSampler SplitEngineGenericMaterial::GetThicknessTexture() const {
  if (!generic_material_parameters_.volume.has_value()) {
    return GetPlaceholderTextureAndSampler();
  }
  return GetTextureAndSampler(*generic_material_parameters_.volume->texture);
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

void SplitEngineGenericMaterial::SetIndexOfRefraction(
    float index_of_refraction) {
  if (!generic_material_parameters_.refraction.has_value()) {
    generic_material_parameters_.refraction.emplace();
  }
  generic_material_parameters_.refraction->index_of_refraction =
      index_of_refraction;
  MarkParametersDirty();
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

void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              bool value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              bool2 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              bool3 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              bool4 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              float value) {
  if (parameter_name == kMetallicFactor) {
    SetMetallicFactor(value);
  } else if (parameter_name == kRoughnessFactor) {
    SetRoughnessFactor(value);
  } else if (parameter_name == kNormalScale) {
    SetNormalScale(value);
  } else if (parameter_name == kSheenRoughnessFactor) {
    SetSheenRoughnessFactor(value);
  } else if (parameter_name == kIndexOfRefraction) {
    SetIndexOfRefraction(value);
  } else if (parameter_name == kThicknessFactor) {
    SetThicknessFactor(value);
  } else if (parameter_name == kAttenuationDistance) {
    SetAttenuationDistance(value);
  } else if (parameter_name == kTransmissionFactor) {
    SetTransmissionFactor(value);
  } else {
    LogUnsupportedParameter(parameter_name);
  }
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              float2 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              float3 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              float4 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              int value) {
  if (parameter_name == kSamplersUvBitflags ||       //
      parameter_name == kBaseColorIndex ||           //
      parameter_name == kMetallicRoughnessIndex ||   //
      parameter_name == kNormalIndex ||              //
      parameter_name == kAoIndex ||                  //
      parameter_name == kEmissiveIndex ||            //
      parameter_name == kClearcoatIndex ||           //
      parameter_name == kClearcoatRoughnessIndex ||  //
      parameter_name == kClearcoatNormalIndex ||     //
      parameter_name == kSheenColorIndex ||          //
      parameter_name == kSheenRoughnessIndex ||      //
      parameter_name == kThicknessIndex ||           //
      parameter_name == kTransmissionIndex) {
    LogParameterNotDirectlyAssignable(parameter_name);
  } else {
    LogUnsupportedParameter(parameter_name);
  }
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              int2 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              int3 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              int4 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              uint value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              uint2 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              uint3 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              uint4 value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              mat3f value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              mat4f value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const bool> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const bool2> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const bool3> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const bool4> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const float> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const float2> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const float3> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const float4> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const int> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const int2> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const int3> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const int4> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const uint> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const uint2> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const uint3> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const uint4> value) {
  LogUnsupportedParameter(parameter_name);
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const mat3f> value) {
  if (parameter_name == kSamplersUvMatrices) {
    LogParameterNotDirectlyAssignable(parameter_name);
  } else {
    LogUnsupportedParameter(parameter_name);
  }
}
void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              absl::Span<const mat4f> value) {
  LogUnsupportedParameter(parameter_name);
}

void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              filament::RgbaType type,
                                              filament::math::float4 color) {
  LogUnsupportedParameter(parameter_name);
}

void SplitEngineGenericMaterial::SetParameter(absl::string_view parameter_name,
                                              filament::RgbType type,
                                              filament::math::float3 color) {
  LogUnsupportedParameter(parameter_name);
}

ABSL_DEPRECATED(
    "Use imp::BorrowedTexturePtr overload instead. See "
    "(broken link).")
void SplitEngineGenericMaterial::SetParameter(
    absl::string_view parameter_name, const imp::Texture* texture,
    std::optional<filament::TextureSampler> sampler_override) {
  LogTextureParameterNotDirectlyAssignable(parameter_name);
}

void SplitEngineGenericMaterial::SetParameter(
    absl::string_view parameter_name, TexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  LogTextureParameterNotDirectlyAssignable(parameter_name);
}

void SplitEngineGenericMaterial::SetParameter(
    absl::string_view parameter_name, OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  LogTextureParameterNotDirectlyAssignable(parameter_name);
}

void SplitEngineGenericMaterial::SetParameter(
    absl::string_view parameter_name, BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  LogTextureParameterNotDirectlyAssignable(parameter_name);
}

bool SplitEngineGenericMaterial::HasParameter(absl::string_view name) {
  // We don't know if the parameter exists because the material is held on the
  // SplitEngine renderer side.
  // NOTE: This means that TrySetParameter will behave the same as SetParameter
  // and crash the app if the parameter does not exist.
  return true;
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

void SplitEngineGenericMaterial::ForEachTexture(
    absl::FunctionRef<void(BorrowedTexturePtr)> fn, SmallSourceLocation loc) {
  for (auto& [_, texture] : borrowed_textures_) {
    if (texture) {
      fn(texture);
    }
  }
}

}  // namespace imp::split_engine
