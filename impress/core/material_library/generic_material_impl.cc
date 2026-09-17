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

#include "core/material_library/generic_material_impl.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/functional/function_ref.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/common/typed_vector.h"
#include "core/config.h"
#include "core/material_library/generic_material_constants.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_package.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/custom_material.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/model/entity_data.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace {

// These are special fallback indices that indicate a fallback sampler should
// be used. We differentiate between color and normal/tangent fallback samplers.
static constexpr int kFallbackSamplerIndexColor = 16;
static constexpr int kFallbackSamplerIndexNormal = 17;

// The list of assignable samplers that exist in Impress's generic glTF
// materials.
static constexpr std::array<absl::string_view, 9> kAssignableSamplers{
    "samplerZero", "samplerOne", "samplerTwo",   "samplerThree", "samplerFour",
    "samplerFive", "samplerSix", "samplerSeven", "samplerEight"};

absl::StatusOr<filament::MaterialInstance*> CreateMaterialInstance(
    const GenericMaterialSpec& spec,
    const MaterialPackage::MaterialCache& materials, absl::string_view name) {
  // Get the filament::Material* for the spec.
  auto it = materials.find(spec);
  if (it == materials.end()) {
    return absl::NotFoundError(
        absl::StrFormat("Material not found: %s", spec.Describe()));
  }
  filament::Material* material = it->second;

  filament::MaterialInstance* material_instance = material->createInstance();
  if (!material_instance) {
    return absl::InternalError("failed to build material");
  }
  return material_instance;
}

}  // namespace

Future<GenericMaterialPtr> GenericMaterialImpl::Create(
    BaseView& view, const GenericMaterialSpec& spec,
    const MaterialPackage::MaterialCache& materials, absl::string_view name) {
  // If running in Split Engine mode, delegate to the backend.
  if (split_engine::SplitEngineSerializer* serializer =
          view.GetSplitEngineSerializer()) {
    return serializer->CreateGenericMaterial(spec);
  }

  absl::StatusOr<filament::MaterialInstance*> material_instance =
      CreateMaterialInstance(spec, materials, name);
  if (!material_instance.ok()) {
    return Future<GenericMaterialPtr>(material_instance.status());
  }
  absl::StatusOr<GenericMaterialImpl::ParameterInfo> parameter_info =
      GetMaterialParameterInfo(*(*material_instance)->getMaterial());
  if (!parameter_info.ok()) {
    return Future<GenericMaterialPtr>(parameter_info.status());
  }
  return Future<GenericMaterialPtr>(absl::WrapUnique(new GenericMaterialImpl(
      view, name, **material_instance, *parameter_info)));
}

GenericMaterialImpl::GenericMaterialImpl(
    BaseView& view, absl::string_view name,
    filament::MaterialInstance& material_instance,
    const ParameterInfo& parameter_info)
    : view_(view),
      material_(new CustomMaterial(
          &material_instance, {},
          view.GetConfig()
              .experimental_feature_flags
              ->enable_filament_material_instance_destruction_check.Value())),
      parameter_info_(parameter_info),
      placeholder_texture_(view.GetTextureFactory().BorrowPlaceholderTexture()),
      placeholder_sampler_() {
  material_->SetName(name);
}

const std::string& GenericMaterialImpl::GetName() const {
  return material_->GetName();
}

void GenericMaterialImpl::SetName(absl::string_view name) {
  material_->SetName(name);
}

const filament::MaterialInstance*
GenericMaterialImpl::GetFilamentMaterialInstance() const {
  return material_->GetFilamentMaterialInstance();
}

filament::MaterialInstance* GenericMaterialImpl::GetFilamentMaterialInstance() {
  return material_->GetFilamentMaterialInstance();
}

TextureAndSampler GenericMaterialImpl::GetBaseColorTexture() const {
  if (auto it = texture_lookup_.find(kBaseColorIndex);
      it != texture_lookup_.end()) {
    return it->second;
  }
  return {nullptr, filament::TextureSampler()};
}

absl::Status GenericMaterialImpl::SetBaseColorUvTransform(
    const mat3f& uv_transform) {
  return AssignSamplerUvTransform(kBaseColorIndex, uv_transform);
}

void GenericMaterialImpl::SetBaseColorFactor(const float4& factor) {
  ApplyMaterialParameter(kBaseColorFactor, factor);
}

float4 GenericMaterialImpl::GetBaseColorFactor() const {
  return material_->GetFilamentMaterialInstance()->getParameter<float4>(
      kBaseColorFactor.data(), kBaseColorFactor.size());
}

TextureAndSampler GenericMaterialImpl::GetMetallicRoughnessTexture() const {
  if (auto it = texture_lookup_.find(kMetallicRoughnessIndex);
      it != texture_lookup_.end()) {
    return it->second;
  }
  return {nullptr, filament::TextureSampler()};
}

absl::Status GenericMaterialImpl::SetMetallicRoughnessUvTransform(
    const mat3f& uv_transform) {
  return AssignSamplerUvTransform(kMetallicRoughnessIndex, uv_transform);
}

void GenericMaterialImpl::SetMetallicFactor(float factor) {
  ApplyMaterialParameter(kMetallicFactor, factor);
}

float GenericMaterialImpl::GetMetallicFactor() const {
  return material_->GetFilamentMaterialInstance()->getParameter<float>(
      kMetallicFactor.data(), kMetallicFactor.size());
}

void GenericMaterialImpl::SetRoughnessFactor(float factor) {
  ApplyMaterialParameter(kRoughnessFactor, factor);
}

float GenericMaterialImpl::GetRoughnessFactor() const {
  return material_->GetFilamentMaterialInstance()->getParameter<float>(
      kRoughnessFactor.data(), kRoughnessFactor.size());
}

TextureAndSampler GenericMaterialImpl::GetNormalTexture() const {
  if (auto it = texture_lookup_.find(kNormalIndex);
      it != texture_lookup_.end()) {
    return it->second;
  }
  return {nullptr, filament::TextureSampler()};
}

absl::Status GenericMaterialImpl::SetNormalUvTransform(
    const mat3f& uv_transform) {
  return AssignSamplerUvTransform(kNormalIndex, uv_transform);
}

void GenericMaterialImpl::SetNormalScale(float scale) {
  ApplyMaterialParameter(kNormalScale, scale);
}

float GenericMaterialImpl::GetNormalScale() const {
  return material_->GetFilamentMaterialInstance()->getParameter<float>(
      kNormalScale.data(), kNormalScale.size());
}

TextureAndSampler GenericMaterialImpl::GetAmbientOcclusionTexture() const {
  if (auto it = texture_lookup_.find(kAoIndex); it != texture_lookup_.end()) {
    return it->second;
  }
  return {nullptr, filament::TextureSampler()};
}

absl::Status GenericMaterialImpl::SetAmbientOcclusionUvTransform(
    const mat3f& uv_transform) {
  return AssignSamplerUvTransform(kAoIndex, uv_transform);
}

void GenericMaterialImpl::SetAmbientOcclusionStrength(float strength) {
  ApplyMaterialParameter(kAoStrength, strength);
}

float GenericMaterialImpl::GetAmbientOcclusionStrength() const {
  return material_->GetFilamentMaterialInstance()->getParameter<float>(
      kAoStrength.data(), kAoStrength.size());
}

TextureAndSampler GenericMaterialImpl::GetEmissiveTexture() const {
  if (auto it = texture_lookup_.find(kEmissiveIndex);
      it != texture_lookup_.end()) {
    return it->second;
  }
  return {nullptr, filament::TextureSampler()};
}

absl::Status GenericMaterialImpl::SetEmissiveUvTransform(
    const mat3f& uv_transform) {
  return AssignSamplerUvTransform(kEmissiveIndex, uv_transform);
}

void GenericMaterialImpl::SetEmissiveFactor(const float3& factor) {
  ApplyMaterialParameter(kEmissiveFactor, factor);
}

float3 GenericMaterialImpl::GetEmissiveFactor() const {
  return material_->GetFilamentMaterialInstance()->getParameter<float3>(
      kEmissiveFactor.data(), kEmissiveFactor.size());
}

TextureAndSampler GenericMaterialImpl::GetClearcoatTexture() const {
  if (auto it = texture_lookup_.find(kClearcoatIndex);
      it != texture_lookup_.end()) {
    return it->second;
  }
  return {placeholder_texture_->GetTexture(), filament::TextureSampler()};
}

TextureAndSampler GenericMaterialImpl::GetClearcoatRoughnessTexture() const {
  if (auto it = texture_lookup_.find(kClearcoatRoughnessIndex);
      it != texture_lookup_.end()) {
    return it->second;
  }
  return {placeholder_texture_->GetTexture(), filament::TextureSampler()};
}

TextureAndSampler GenericMaterialImpl::GetClearcoatNormalTexture() const {
  if (auto it = texture_lookup_.find(kClearcoatNormalIndex);
      it != texture_lookup_.end()) {
    return it->second;
  }
  return {placeholder_texture_->GetTexture(), filament::TextureSampler()};
}

void GenericMaterialImpl::SetClearcoatFactors(const float3& factor) {
  ApplyMaterialParameter(kClearcoatRoughnessNormalFactors, factor);
}

TextureAndSampler GenericMaterialImpl::GetSheenColorTexture() const {
  if (auto it = texture_lookup_.find(kSheenColorIndex);
      it != texture_lookup_.end()) {
    return it->second;
  }
  return {placeholder_texture_->GetTexture(), filament::TextureSampler()};
}

void GenericMaterialImpl::SetSheenColorFactor(const float3& factor) {
  ApplyMaterialParameter(kSheenColorFactor, factor);
}

TextureAndSampler GenericMaterialImpl::GetSheenRoughnessTexture() const {
  if (auto it = texture_lookup_.find(kSheenRoughnessIndex);
      it != texture_lookup_.end()) {
    return it->second;
  }

  return {placeholder_texture_->GetTexture(), filament::TextureSampler()};
}

void GenericMaterialImpl::SetSheenRoughnessFactor(float factor) {
  ApplyMaterialParameter(kSheenRoughnessFactor, factor);
}

void GenericMaterialImpl::SetIndexOfRefraction(float index_of_refraction) {
  ApplyMaterialParameter(kIndexOfRefraction, index_of_refraction);
}

TextureAndSampler GenericMaterialImpl::GetThicknessTexture() const {
  if (auto it = texture_lookup_.find(kThicknessIndex);
      it != texture_lookup_.end()) {
    return it->second;
  }
  return {placeholder_texture_->GetTexture(), filament::TextureSampler()};
}

void GenericMaterialImpl::SetThicknessFactor(float factor) {
  ApplyMaterialParameter(kThicknessFactor, factor);
}

void GenericMaterialImpl::SetAttenuationDistance(float distance) {
  ApplyMaterialParameter(kAttenuationDistance, distance);
}

void GenericMaterialImpl::SetAttenuationColor(const float3& color) {
  ApplyMaterialParameter(kAttenuationColor, color);
}
TextureAndSampler GenericMaterialImpl::GetTransmissionTexture() const {
  if (auto it = texture_lookup_.find(kTransmissionIndex);
      it != texture_lookup_.end()) {
    return it->second;
  }
  return {placeholder_texture_->GetTexture(), filament::TextureSampler()};
}

absl::Status GenericMaterialImpl::SetTransmissionUvTransform(
    const mat3f& uv_transform) {
  return AssignSamplerUvTransform(kTransmissionIndex, uv_transform);
}
void GenericMaterialImpl::SetTransmissionFactor(float factor) {
  ApplyMaterialParameter(kTransmissionFactor, factor);
}

void GenericMaterialImpl::SetAlphaCutoff(float alpha_cutoff) {
  if (material_->GetFilamentMaterialInstance()
          ->getMaterial()
          ->getBlendingMode() == filament::Material::BlendingMode::MASKED) {
    material_->GetFilamentMaterialInstance()->setMaskThreshold(alpha_cutoff);
  }
}

float GenericMaterialImpl::GetAlphaCutoff() const {
  return material_->GetFilamentMaterialInstance()->getMaskThreshold();
}

std::optional<TextureAndSampler> GenericMaterialImpl::GetFeatureIdTexture(
    int index) const {
  if (index < 0 || index >= kFeatureIdTextureNames.size()) {
    return std::nullopt;
  }

  const absl::string_view feature_id_texture_name =
      kFeatureIdTextureNames[index];
  if (auto it = texture_lookup_.find(feature_id_texture_name);
      it != texture_lookup_.end()) {
    return it->second;
  }
  return std::nullopt;
}

template <typename T>
void GenericMaterialImpl::ApplyMaterialParameter(absl::string_view name,
                                                 const T& value) {
  material_->GetFilamentMaterialInstance()->setParameter(name.data(),
                                                         name.size(), value);
  parameters_.emplace_back(model::MaterialParameter(name, value));
}

template <>
void GenericMaterialImpl::ApplyMaterialParameter<std::vector<mat3f>>(
    absl::string_view name, const std::vector<mat3f>& value) {
  // The size of the mat3 array should match the number of samplers available
  // and may be truncated if too many samplers are requested.
  size_t mat_size = value.size() < parameter_info_.max_available_samplers
                        ? value.size()
                        : parameter_info_.max_available_samplers;
  if (!value.empty()) {
    material_->GetFilamentMaterialInstance()->setParameter(
        name.data(), name.size(), value.data(), mat_size);
  }
  parameters_.emplace_back(model::MaterialParameter(
      name, std::vector<mat3f>(value.begin(), value.end())));
}

GenericMaterialPtr GenericMaterialImpl::Duplicate() const {
  // TODO: (broken link) - Make it possible to duplicate imp::Material to avoid
  // losing owned/borrowed texture tracking.
  if (HoldsOwnedTextures()) {
    IMP_LOG(imp::FATAL) << "Cannot duplicate material with owned textures.";
  }

  filament::MaterialInstance* duplicate_instance =
      filament::MaterialInstance::duplicate(
          material_->GetFilamentMaterialInstance());
  if (auto* serializer = view_.GetSplitEngineSerializer()) {
    serializer->DuplicateMaterialInstance(
        material_->GetFilamentMaterialInstance(), duplicate_instance);
  }
  auto result = absl::WrapUnique(new GenericMaterialImpl(
      view_, material_->GetName(), *duplicate_instance, parameter_info_));
  result->parameters_ = parameters_;
  result->material_textures_ = material_textures_;
  result->sampler_index_lookup_ = sampler_index_lookup_;
  result->texture_lookup_ = texture_lookup_;
  result->next_available_assignable_sampler_index_ =
      next_available_assignable_sampler_index_;
  result->samplers_uv_bitflags_ = samplers_uv_bitflags_;
  result->samplers_uv_matrices_ = samplers_uv_matrices_;
  return result;
}

absl::Status GenericMaterialImpl::AssignTexturesAndParams(
    const GenericMaterialParameters& generic_material_parameters,
    const TextureBorrower& texture_borrower) {
  if (parameter_info_.sampler_parameters.empty()) {
    // This is the depth material, it doesn't need any parameters set.
    return absl::OkStatus();
  }
  if (generic_material_parameters.base_color) {
    MP_RETURN_IF_ERROR(
        AssignTexture(texture_borrower, kBaseColorIndex,
                      generic_material_parameters.base_color->texture));
    SetBaseColorFactor(generic_material_parameters.base_color->factor);
  } else {
    AssignFallbackSampler(kBaseColorIndex);
    SetBaseColorFactor(kDefaultBaseColorFactor);
  }

  if (material_->GetFilamentMaterialInstance()->getMaterial()->getShading() ==
      filament::Material::Shading::LIT) {
    if (generic_material_parameters.metallic_roughness) {
      MP_RETURN_IF_ERROR(AssignTexture(
          texture_borrower, kMetallicRoughnessIndex,
          generic_material_parameters.metallic_roughness->texture));
      SetMetallicFactor(
          generic_material_parameters.metallic_roughness->metallic_factor);
      SetRoughnessFactor(
          generic_material_parameters.metallic_roughness->roughness_factor);
    } else {
      AssignFallbackSampler(kMetallicRoughnessIndex);
      SetMetallicFactor(kDefaultMetallicFactor);
      SetRoughnessFactor(kDefaultRoughnessFactor);
    }
    if (generic_material_parameters.normal) {
      MP_RETURN_IF_ERROR(
          AssignTexture(texture_borrower, kNormalIndex,
                        generic_material_parameters.normal->texture));
      SetNormalScale(generic_material_parameters.normal->factor);
    } else {
      AssignFallbackSampler(kNormalIndex, FallbackSampler::kNormal);
      SetNormalScale(kDefaultNormalFactor);
    }

    if (generic_material_parameters.ambient_occlusion) {
      MP_RETURN_IF_ERROR(AssignTexture(
          texture_borrower, kAoIndex,
          generic_material_parameters.ambient_occlusion->texture));
      SetAmbientOcclusionStrength(
          generic_material_parameters.ambient_occlusion->factor);
    } else {
      AssignFallbackSampler(kAoIndex);
      SetAmbientOcclusionStrength(kDefaultAmbientOcclusionFactor);
    }

    if (generic_material_parameters.emissive) {
      MP_RETURN_IF_ERROR(
          AssignTexture(texture_borrower, kEmissiveIndex,
                        generic_material_parameters.emissive->texture));
      SetEmissiveFactor(generic_material_parameters.emissive->factor);
    } else {
      AssignFallbackSampler(kEmissiveIndex);
      SetEmissiveFactor(kDefaultEmissiveFactor);
    }

    if (generic_material_parameters.clearcoat) {
      MP_RETURN_IF_ERROR(AssignTexture(
          texture_borrower, kClearcoatIndex,
          generic_material_parameters.clearcoat->intensity_texture));
      MP_RETURN_IF_ERROR(AssignTexture(
          texture_borrower, kClearcoatRoughnessIndex,
          generic_material_parameters.clearcoat->roughness_texture));
      MP_RETURN_IF_ERROR(
          AssignTexture(texture_borrower, kClearcoatNormalIndex,
                        generic_material_parameters.clearcoat->normal_texture));
      SetClearcoatFactors(generic_material_parameters.clearcoat->factor);
    } else {
      AssignFallbackSampler(kClearcoatIndex);
      AssignFallbackSampler(kClearcoatRoughnessIndex);
      AssignFallbackSampler(kClearcoatNormalIndex, FallbackSampler::kNormal);
      SetClearcoatFactors(kDefaultClearcoatFactor);
    }

    if (generic_material_parameters.sheen) {
      MP_RETURN_IF_ERROR(
          AssignTexture(texture_borrower, kSheenColorIndex,
                        generic_material_parameters.sheen->color_texture));
      SetSheenColorFactor(generic_material_parameters.sheen->color_factor);
      MP_RETURN_IF_ERROR(
          AssignTexture(texture_borrower, kSheenRoughnessIndex,
                        generic_material_parameters.sheen->roughness_texture));
      SetSheenRoughnessFactor(
          generic_material_parameters.sheen->roughness_factor);
    } else {
      AssignFallbackSampler(kSheenColorIndex);
      AssignFallbackSampler(kSheenRoughnessIndex);
      SetSheenColorFactor(kDefaultSheenColorFactor);
      SetSheenRoughnessFactor(kDefaultSheenRoughnessFactor);
    }

    if (generic_material_parameters.volume) {
      MP_RETURN_IF_ERROR(
          AssignTexture(texture_borrower, kThicknessIndex,
                        generic_material_parameters.volume->thickness_texture));
      SetThicknessFactor(generic_material_parameters.volume->thickness_factor);
      SetAttenuationDistance(
          generic_material_parameters.volume->attenuation_distance);
      SetAttenuationColor(
          generic_material_parameters.volume->attenuation_color);
    } else {
      AssignFallbackSampler(kThicknessIndex);
      SetThicknessFactor(kDefaultThicknessFactor);
      SetAttenuationDistance(kDefaultAttenuationDistance);
      SetAttenuationColor(kDefaultAttenuationColor);
    }

    if (generic_material_parameters.transmission) {
      MP_RETURN_IF_ERROR(
          AssignTexture(texture_borrower, kTransmissionIndex,
                        generic_material_parameters.transmission->texture));
      SetTransmissionFactor(generic_material_parameters.transmission->factor);
    } else {
      AssignFallbackSampler(kTransmissionIndex);
      SetTransmissionFactor(kDefaultTransmissionFactor);
    }

    if (generic_material_parameters.refraction) {
      SetIndexOfRefraction(
          generic_material_parameters.refraction->index_of_refraction);
    } else {
      SetIndexOfRefraction(kDefaultIndexOfRefraction);
    }
  }

  if (generic_material_parameters.feature_id_textures.has_value()) {
    int feature_id_index = 0;
    for (const GenericMaterialTextureParameter& feature_id_texture :
         *generic_material_parameters.feature_id_textures) {
      const BorrowedTexturePtr texture = texture_borrower(
          feature_id_texture.texture_id, SmallSourceLocation::Current());
      if (!texture) {
        return absl::NotFoundError(absl::StrFormat(
            "feature_id_texture not found: %d", feature_id_texture.texture_id));
      }
      texture_lookup_.insert_or_assign<TextureAndSampler>(
          kFeatureIdTextureNames[feature_id_index++],
          {texture->GetTexture(), feature_id_texture.sampler,
           feature_id_texture.uv_transform});
      if (feature_id_index == kFeatureIdTextureNames.size()) {
        break;
      }
    }
  }

  ApplyMaterialParameter(kSamplersUvBitflags, samplers_uv_bitflags_);
  ApplyMaterialParameter(kSamplersUvMatrices, std::move(samplers_uv_matrices_));
  if (material_->GetFilamentMaterialInstance()
          ->getMaterial()
          ->getBlendingMode() == filament::Material::BlendingMode::MASKED) {
    if (generic_material_parameters.masking) {
      SetAlphaCutoff(generic_material_parameters.masking->alpha_cutoff);
    } else {
      SetAlphaCutoff(kDefaultAlphaCutoff);
    }
  }

  // If any texture sampler slots are unused, assign the placeholder texture.
  AssignPlaceholderTexturesToUnusedSamplers();

  return absl::OkStatus();
}

std::vector<model::MaterialParameter> GenericMaterialImpl::GetParameters()
    const {
  return parameters_;
}

TypedVector<model::MaterialTexture> GenericMaterialImpl::GetTextures() const {
  return material_textures_;
}

StringMap<int> GenericMaterialImpl::GetSamplerIndexLookup() const {
  return sampler_index_lookup_;
}

absl::string_view GenericMaterialImpl::GetParameterTransformName(
    absl::string_view sampler_name) const {
  return material_->GetParameterTransformName(sampler_name);
}

Material::HeldTextureType GenericMaterialImpl::GetAssignedTextureType(
    absl::string_view parameter_name) {
  return material_->GetAssignedTextureType(parameter_name);
}

imp::StringMap<const filament::Texture*>
GenericMaterialImpl::GetUnownedFilamentTextures() const {
  return material_->GetUnownedFilamentTextures();
}

bool GenericMaterialImpl::HoldsOwnedTextures() const {
  return material_->HoldsOwnedTextures();
}

void GenericMaterialImpl::ForEachTexture(
    absl::FunctionRef<void(BorrowedTexturePtr)> fn, SmallSourceLocation loc) {
  material_->ForEachTexture(fn, loc);
}

BorrowedMaterialPtr GenericMaterialImpl::GetMaterialInternal(
    SmallSourceLocation loc) const {
  return material_.Borrow(loc);
}

absl::Status GenericMaterialImpl::SetTextureForChannel(
    absl::string_view channel_name, OwnedOrBorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannelInternal(channel_name, std::move(texture),
                                      sampler_override, kIdentityMat3f,
                                      /*uses_uv1=*/false);
}

absl::Status GenericMaterialImpl::AssignTexture(
    const TextureBorrower& texture_borrower,
    absl::string_view texture_channel_name,
    const absl::optional<GenericMaterialTextureParameter>& texture_parameter) {
  if (!texture_parameter) {
    // Some parameters are optional.
    AssignFallbackSampler(texture_channel_name,
                          GetFallbackForChannel(texture_channel_name));
    return absl::OkStatus();
  }

  const BorrowedTexturePtr texture = texture_borrower(
      texture_parameter->texture_id, SmallSourceLocation::Current());
  if (!texture) {
    return absl::NotFoundError(absl::StrFormat("Texture not found: %d",
                                               texture_parameter->texture_id));
  }

  return SetTextureForChannelInternal(
      texture_channel_name, texture, texture_parameter->sampler,
      texture_parameter->uv_transform, texture_parameter->uses_uv1);
}

absl::Status GenericMaterialImpl::SetTextureForChannelInternal(
    absl::string_view channel_name, OwnedOrBorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override,
    const mat3f& uv_transform, bool uses_uv1) {
  if (!texture) {
    auto it = sampler_index_lookup_.find(channel_name);
    if (it != sampler_index_lookup_.end()) {
      int sampler_index = it->second;
      if (sampler_index < kAssignableSamplers.size()) {
        absl::string_view sampler_name = kAssignableSamplers[sampler_index];
        AssignPlaceholderTexture(sampler_name);
        samplers_uv_matrices_[sampler_index] = kIdentityMat3f;
        samplers_uv_bitflags_ &= ~(1 << sampler_index);
        ApplyMaterialParameter(kSamplersUvBitflags,
                               static_cast<int32_t>(samplers_uv_bitflags_));
        ApplyMaterialParameter(kSamplersUvMatrices, samplers_uv_matrices_);
        AssignFallbackSampler(channel_name,
                              GetFallbackForChannel(channel_name));
        texture_lookup_.erase(channel_name);
      }
    }

    return absl::OkStatus();
  }

  int sampler_index;
  auto it = sampler_index_lookup_.find(channel_name);

  if (it != sampler_index_lookup_.end()) {
    sampler_index = it->second;
    samplers_uv_matrices_[sampler_index] = uv_transform;
  } else {
    sampler_index = next_available_assignable_sampler_index_;
    next_available_assignable_sampler_index_++;
    samplers_uv_matrices_.push_back(uv_transform);
  }

  if (sampler_index >= kAssignableSamplers.size()) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Sampler index %d is out of range [0, %d)",
                        sampler_index, kAssignableSamplers.size() - 1));
  }
  absl::string_view sampler_name = kAssignableSamplers[sampler_index];

  if (IsValidSampler(sampler_name)) {
    filament::Texture* filament_texture = texture->GetTexture();
    filament::TextureSampler sampler =
        sampler_override ? *sampler_override : texture->GetSampler();

    std::variant<BorrowedTexturePtr, OwnedTexturePtr> texture_variant =
        texture.Extract();
    if (std::holds_alternative<OwnedTexturePtr>(texture_variant)) {
      material_->SetParameter(
          sampler_name, std::move(std::get<OwnedTexturePtr>(texture_variant)),
          sampler_override);
    } else {
      material_->SetParameter(sampler_name,
                              std::get<BorrowedTexturePtr>(texture_variant),
                              sampler_override);
    }

    texture_lookup_.insert_or_assign<TextureAndSampler>(
        channel_name, {filament_texture, sampler, uv_transform});

    material_->SetParameter(channel_name, static_cast<int32_t>(sampler_index));
    sampler_index_lookup_[channel_name.data()] = sampler_index;

    if (uses_uv1) {
      samplers_uv_bitflags_ |= 1 << sampler_index;
    } else {
      samplers_uv_bitflags_ &= ~(1 << sampler_index);
    }

    ApplyMaterialParameter(kSamplersUvBitflags,
                           static_cast<int32_t>(samplers_uv_bitflags_));
    ApplyMaterialParameter(kSamplersUvMatrices, samplers_uv_matrices_);

    // TODO: (broken link) - Remove all of this once MaterialConfig is removed.
    parameters_.emplace_back(
        model::MaterialParameter(channel_name, sampler_index));
    parameters_.emplace_back(model::MaterialParameter(
        sampler_name, TextureAndSampler(filament_texture, sampler)));
  } else {
    // Use a fallback sampler (if specified) if the sampler is invalid.
    // This can happen if too many samplers are requested.
    const auto sampler_fallback_index =
        GetFallbackSampleIndex(GetFallbackForChannel(channel_name));
    material_->SetParameter(channel_name, sampler_fallback_index);

    // TODO: (broken link) - Remove all of this once MaterialConfig is removed.
    parameters_.emplace_back(
        model::MaterialParameter{channel_name, sampler_fallback_index});
    sampler_index_lookup_.emplace(channel_name, sampler_fallback_index);
  }
  return absl::OkStatus();
}

absl::Status GenericMaterialImpl::AssignSamplerUvTransform(
    absl::string_view sampler_name, const mat3f& uv_transform) {
  auto it = sampler_index_lookup_.find(sampler_name);
  if (it == sampler_index_lookup_.end()) {
    return absl::NotFoundError(
        absl::StrFormat("Sampler not found: %s", sampler_name));
  }
  samplers_uv_matrices_[it->second] = uv_transform;
  // It is safe to update the texture_lookup since we know the key exists.
  texture_lookup_.at(sampler_name).uv_transform = uv_transform;
  ApplyMaterialParameter(kSamplersUvMatrices, samplers_uv_matrices_);
  return absl::OkStatus();
}

GenericMaterialImpl::FallbackSampler GenericMaterialImpl::GetFallbackForChannel(
    absl::string_view channel_name) {
  if (channel_name == kNormalIndex || channel_name == kClearcoatNormalIndex) {
    return FallbackSampler::kNormal;
  }
  return FallbackSampler::kWhite;
}

// Converts the FallbackSampler enum to the index of the fallback sampler in
// the generic glTF material.
int GenericMaterialImpl::GetFallbackSampleIndex(
    FallbackSampler fallback_sample) {
  switch (fallback_sample) {
    case FallbackSampler::kWhite:
      return kFallbackSamplerIndexColor;
    case FallbackSampler::kNormal:
      return kFallbackSamplerIndexNormal;
    default:
      IMP_LOG(imp::WARNING) << "Invalid fallback sample specified, defaulting to "
                      "white fallback sampler.";
      return kFallbackSamplerIndexColor;
  }
}

// Assigns a fallback sampler to use for usages that don't have any texture
// assigned to them in the glTf file.
void GenericMaterialImpl::AssignFallbackSampler(
    absl::string_view index_parameter_name, FallbackSampler fallback_sample) {
  ApplyMaterialParameter(index_parameter_name,
                         GetFallbackSampleIndex(fallback_sample));
  parameters_.push_back(model::MaterialParameter(
      index_parameter_name, GetFallbackSampleIndex(fallback_sample)));
}

void GenericMaterialImpl::AssignPlaceholderTexture(
    absl::string_view sampler_name) {
  if (!IsValidSampler(sampler_name)) return;

  material_->GetFilamentMaterialInstance()->setParameter(
      sampler_name.data(), sampler_name.size(),
      placeholder_texture_->GetTexture(), placeholder_sampler_);
  // TODO: (broken link) - Remove all of this once MaterialConfig is removed.
  parameters_.push_back(model::MaterialParameter(
      sampler_name, TextureAndSampler(placeholder_texture_->GetTexture(),
                                      placeholder_sampler_)));
}

void GenericMaterialImpl::AssignPlaceholderTexturesToUnusedSamplers() {
  int index = next_available_assignable_sampler_index_;
  while (index < parameter_info_.max_available_samplers) {
    AssignPlaceholderTexture(kAssignableSamplers[index]);
    index++;
  }

  // These texture samplers don't exist for lite materials.
  if (parameter_info_.has_estimated_depth_texture) {
    AssignPlaceholderTexture(kEstimatedDepthTexture);
  }

  // On Android, this is an external texture so we don't need to bind
  // a placeholder texture to it.
#if !IMP_PLATFORM(ANDROID)
  if (parameter_info_.has_camera_texture) {
    AssignPlaceholderTexture(kCameraTexture);
  }
#endif
}

bool GenericMaterialImpl::IsValidSampler(absl::string_view sampler_name) {
  return material_->GetFilamentMaterialInstance()->getMaterial()->hasParameter(
             std::string(sampler_name).c_str()) &&
         (parameter_info_.sampler_parameters.find(sampler_name) !=
          parameter_info_.sampler_parameters.end());
}

absl::StatusOr<GenericMaterialImpl::ParameterInfo>
GenericMaterialImpl::GetMaterialParameterInfo(
    const filament::Material& material) {
  GenericMaterialImpl::ParameterInfo parameter_info;
  const size_t parameter_count = material.getParameterCount();
  // Early exit if there are no parameters to avoid calling
  // material `filament::Material::getParameters` with a null parameters
  // pointer.
  if (parameter_count == 0) {
    return parameter_info;
  }

  parameter_info.material_parameters.resize(parameter_count);
  const size_t actual_parameter_count =
      material.getParameters(parameter_info.material_parameters.data(),
                             parameter_info.material_parameters.size());
  parameter_info.material_parameters.resize(actual_parameter_count);
  if (parameter_info.material_parameters.empty()) {
    return parameter_info;
  }

  for (const auto& parameter : parameter_info.material_parameters) {
    if (parameter.isSampler) {
      parameter_info.sampler_parameters.insert(parameter.name);
    }

    std::string parameter_name(parameter.name);
    if (parameter_name == kSamplersUvMatrices) {
      parameter_info.max_available_samplers = parameter.count;
    } else if (parameter_name == kEstimatedDepthTexture) {
      parameter_info.has_estimated_depth_texture = true;
    } else if (parameter_name == kCameraTexture) {
      parameter_info.has_camera_texture = true;
    }
  }

  // If there are samplers, there should be a uv matrix parameter.
  if (!parameter_info.sampler_parameters.empty() &&
      parameter_info.max_available_samplers == 0) {
    return absl::NotFoundError(
        absl::StrFormat("%s parameter not found in the material - this is "
                        "required to get the size of the assignable samplers.",
                        kSamplersUvMatrices));
  }
  return parameter_info;
}

}  // namespace imp
