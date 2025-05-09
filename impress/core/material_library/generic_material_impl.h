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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_IMPL_H_

#include <cstdint>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "core/async/future.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/common/typed_vector.h"
#include "core/material_library/generic_material.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_package.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"
#include "core/view/utils/string_set.h"

namespace imp {

// Concrete implementation of GenericMaterial.
//
// Note: This class delegates to the SplitEngineSerializer if it is available
// on the BaseView. This class will then be created on the backend side to
// manage the real material instance.
//
// TODO: (broken link) - Add scuba tests for GenericMaterial independent of glTF.
class GenericMaterialImpl : public GenericMaterial {
 public:
  // Creates a GenericMaterial from a GenericMaterial schema.
  // Note: The material cache must already include a pre-loaded material that
  // matches the schema for this to succeed. Additionally, all textures required
  // by the material must be loaded and present in the textures vector.
  static Future<GenericMaterialPtr> Create(
      BaseView& view, const GenericMaterialSpec& spec,
      const MaterialPackage::MaterialCache& materials,
      absl::string_view name = "GenericMaterial");

  ~GenericMaterialImpl();

  GenericMaterialPtr Duplicate() const override;

  absl::Status AssignTexturesAndParams(
      const GenericMaterialParameters& generic_material_parameters,
      const TextureBorrower& texture_borrower) override;
  // TODO: Remove this once we fully migrate to TextureBorrower.
  absl::Status AssignTexturesAndParams(
      const GenericMaterialParameters& generic_material_parameters,
      const TextureProvider& texture_provider) override;

  absl::string_view GetName() const override;
  std::vector<MaterialParameter> GetParameters() const override;
  TypedVector<MaterialTexture> GetTextures() const override;
  StringMap<int> GetSamplerIndexLookup() const override;

  TextureAndSampler GetBaseColorTexture() const override;
  absl::Status SetBaseColorUvTransform(const mat3f& uv_transform) override;
  void SetBaseColorFactor(const float4& factor) override;
  TextureAndSampler GetMetallicRoughnessTexture() const override;
  absl::Status SetMetallicRoughnessUvTransform(
      const mat3f& uv_transform) override;
  void SetMetallicFactor(float factor) override;
  void SetRoughnessFactor(float factor) override;
  TextureAndSampler GetNormalTexture() const override;
  absl::Status SetNormalUvTransform(const mat3f& uv_transform) override;
  void SetNormalScale(float scale) override;
  TextureAndSampler GetAmbientOcclusionTexture() const override;
  absl::Status SetAmbientOcclusionUvTransform(
      const mat3f& uv_transform) override;
  void SetAmbientOcclusionStrength(float strength) override;
  TextureAndSampler GetEmissiveTexture() const override;
  absl::Status SetEmissiveUvTransform(const mat3f& uv_transform) override;
  void SetEmissiveFactor(const float3& factor) override;
  TextureAndSampler GetClearcoatTexture() const override;
  TextureAndSampler GetClearcoatNormalTexture() const override;
  TextureAndSampler GetClearcoatRoughnessTexture() const override;
  void SetClearcoatFactors(const float3& factor) override;
  TextureAndSampler GetSheenColorTexture() const override;
  void SetSheenColorFactor(const float3& factor) override;
  TextureAndSampler GetSheenRoughnessTexture() const override;
  void SetSheenRoughnessFactor(float factor) override;
  TextureAndSampler GetTransmissionTexture() const override;
  absl::Status SetTransmissionUvTransform(const mat3f& uv_transform) override;
  void SetTransmissionFactor(float factor) override;
  void SetIndexOfRefraction(float index_of_refraction) override;
  void SetAlphaCutoff(float alpha_cutoff) override;

 protected:
  BorrowedMaterialPtr GetMaterialInternal(
      SmallSourceLocation loc) const override;

 private:
  // Information about the parameters and samplers in the material.
  struct ParameterInfo {
    std::vector<filament::Material::ParameterInfo> material_parameters;
    StringSet sampler_parameters;
    int max_available_samplers;
    bool has_estimated_depth_texture = false;
    bool has_camera_texture = false;
  };

  GenericMaterialImpl(BaseView& view, absl::string_view name,
                      filament::MaterialInstance& material_instance,
                      const ParameterInfo& parameter_info);

  // Queries filament for all parameters and samplers in the material.
  static absl::StatusOr<ParameterInfo> GetMaterialParameterInfo(
      const filament::Material& material);

  // Applies a material parameter of the given name and generic value.
  // This works for all types except for textures.
  template <typename T>
  void ApplyMaterialParameter(absl::string_view name, const T& value);

  // Determines which fallback sampler to use if a texture is not assigned to a
  // texture channel in the glTF file.
  enum class FallbackSampler { kWhite, kNormal };

  // Applies a texture parameter to the material at the given sampler index.
  // The texture index is used to look up the texture in the textures_ vector.
  // The fallback_sample is used if this texture channel has no texture assigned
  // to it in the glTF file.
  absl::Status ApplyMaterialTextureParameter(
      const TextureBorrower& texture_borrower, uint16_t sampler_index,
      absl::string_view texture_channel_name,
      const GenericMaterialTextureParameter& texture_parameter,
      FallbackSampler fallback_sample);
  // TODO: Remove this once we fully migrate to TextureBorrower.
  absl::Status ApplyMaterialTextureParameter(
      const TextureProvider& texture_provider, uint16_t sampler_index,
      absl::string_view texture_channel_name,
      const GenericMaterialTextureParameter& texture_parameter,
      FallbackSampler fallback_sample);

  // Assigns the texture for the given texture channel (i.e. kBaseColorIndex) to
  // the next available sampler in the material.
  //
  // Generic glTF materials contain sets of samplers that can be assigned for
  // different usages by index. This allows samplers to be re-used for different
  // purposes, allowing the glTF material to be more scalable as glTF extensions
  // are added that require new textures. i.e. a new extension can be added that
  // re-uses an existing sampler.
  //
  // GenericMaterial tracks what assignable samplers are currently available in
  // the material, assigns each required channel to a sampler, and then sets a
  // placeholder texture to any unused samplers.
  absl::Status AssignTexture(
      const TextureBorrower& texture_borrower,
      absl::string_view texture_channel_name,
      const absl::optional<GenericMaterialTextureParameter>& texture_parameter,
      FallbackSampler fallback_sample = FallbackSampler::kWhite);
  // TODO: Remove this once we fully migrate to TextureBorrower.
  absl::Status AssignTexture(
      const TextureProvider& texture_provider,
      absl::string_view texture_channel_name,
      const absl::optional<GenericMaterialTextureParameter>& texture_parameter,
      FallbackSampler fallback_sample = FallbackSampler::kWhite);

  // Sets the UV transform for the sampler with the given name, if present.
  absl::Status AssignSamplerUvTransform(absl::string_view sampler_name,
                                        const mat3f& uv_transform);

  int GetFallbackSampleIndex(FallbackSampler fallback_sample);

  // Assigns a fallback sample to use for usages that don't have any texture
  // assigned to them in the glTf file.
  void AssignFallbackSampler(
      absl::string_view index_parameter_name,
      FallbackSampler fallback_sample = FallbackSampler::kWhite);

  // Assigns a placeholder texture to the sampler with the given name.
  void AssignPlaceholderTexture(absl::string_view sampler_name);

  // Assigns a fallback sample to use for samplers that don't have any texture
  // assigned to them in the glTf file.
  void AssignPlaceholderTexturesToUnusedSamplers();

  // Returns the number of available samplers based on the material and the
  // loader options.
  //
  // Different versions of the compiled generic glTF material have different
  // numbers of available assignable samplers.
  int GetMaxAvailableSamplers();

  bool IsValidSampler(absl::string_view sampler_name);

  BaseView& view_;
  std::string name_;
  OwnedMaterialPtr material_;
  ParameterInfo parameter_info_;
  BorrowedTexturePtr placeholder_texture_;
  filament::TextureSampler placeholder_sampler_;

  // TODO (broken link) - Remove name_, parameters_, material_textures_, and
  // sampler_index_lookup_ once MaterialConfig is removed.
  std::vector<MaterialParameter> parameters_;
  TypedVector<MaterialTexture> material_textures_;
  StringMap<int> sampler_index_lookup_;

  // New system for storing texture data for easy retrieval, i.e. can get the
  // texture/sampler for a given texture channel name such as kBaseColorIndex.
  RobinMap<absl::string_view, TextureAndSampler> texture_lookup_;

  int next_available_assignable_sampler_index_ = 0;
  int samplers_uv_bitflags_ = 0;
  std::vector<mat3f> samplers_uv_matrices_;
};

template <typename T>
void GenericMaterialImpl::ApplyMaterialParameter(absl::string_view name,
                                                 const T& value) {
  material_->GetFilamentMaterialInstance()->setParameter(name.data(), value);
  parameters_.emplace_back(MaterialParameter{name, value});
}

template <>
void GenericMaterialImpl::ApplyMaterialParameter(absl::string_view name,
                                                 const std::vector<mat3f>& v);
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_IMPL_H_
