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
#include <optional>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/functional/function_ref.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
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
#include "core/model/entity_data.h"
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

  const std::string& GetName() const override;
  void SetName(absl::string_view name) override;

  absl::Status AssignTexturesAndParams(
      const GenericMaterialParameters& generic_material_parameters,
      const TextureBorrower& texture_borrower) override;

  const filament::MaterialInstance* GetFilamentMaterialInstance()
      const override;
  filament::MaterialInstance* GetFilamentMaterialInstance() override;
  std::vector<model::MaterialParameter> GetParameters() const override;
  TypedVector<model::MaterialTexture> GetTextures() const override;
  StringMap<int> GetSamplerIndexLookup() const override;

  TextureAndSampler GetBaseColorTexture() const override;
  absl::Status SetBaseColorUvTransform(const mat3f& uv_transform) override;
  // Note: If you are using GenericMaterial on non glTF models (e.g., a simple
  // quad with color), you should set the vertex color to white before calling
  // `SetBaseColorFactor`. This is because GenericMaterial *requires* that
  // mesh has vertex color attributes. See
  // google3/third_party/impress/core/loader/data/generic_material_unlit.mat.template.glsl
  // Vertex color can be set via QuadSettings.
  // For example:
  // imp::CreateQuadSettings quad_settings{
  //     .color = kWhite,
  //     // your other settings
  //     .size = {1.0f, 1.0f}
  // };
  // auto render_component =
  //     quad_video_node_->AddComponent<imp::MeshRenderer>();
  // render_component->SetMesh(
  //     GetView().GetMeshFactory().CreatePanel(quad_settings));
  // imp::GenericMaterialImpl::Create(...)
  //     .Then([](GenericMaterialPtr material) {
  //   material->SetBaseColorFactor(your_color);
  // });
  void SetBaseColorFactor(const float4& factor) override;
  float4 GetBaseColorFactor() const override;

  TextureAndSampler GetMetallicRoughnessTexture() const override;
  absl::Status SetMetallicRoughnessUvTransform(
      const mat3f& uv_transform) override;
  void SetMetallicFactor(float factor) override;
  float GetMetallicFactor() const override;
  void SetRoughnessFactor(float factor) override;
  float GetRoughnessFactor() const override;

  TextureAndSampler GetNormalTexture() const override;
  absl::Status SetNormalUvTransform(const mat3f& uv_transform) override;
  void SetNormalScale(float scale) override;
  float GetNormalScale() const override;

  TextureAndSampler GetAmbientOcclusionTexture() const override;
  absl::Status SetAmbientOcclusionUvTransform(
      const mat3f& uv_transform) override;
  void SetAmbientOcclusionStrength(float strength) override;
  float GetAmbientOcclusionStrength() const override;

  TextureAndSampler GetEmissiveTexture() const override;
  absl::Status SetEmissiveUvTransform(const mat3f& uv_transform) override;
  void SetEmissiveFactor(const float3& factor) override;
  float3 GetEmissiveFactor() const override;

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
  TextureAndSampler GetThicknessTexture() const override;
  void SetThicknessFactor(float factor) override;
  void SetAttenuationDistance(float distance) override;
  void SetAttenuationColor(const float3& color) override;
  void SetIndexOfRefraction(float index_of_refraction) override;

  std::optional<TextureAndSampler> GetFeatureIdTexture(
      int index) const override;

  void SetAlphaCutoff(float alpha_cutoff) override;
  float GetAlphaCutoff() const override;

  void SetParameter(absl::string_view parameter_name, bool value) override;
  void SetParameter(absl::string_view parameter_name, bool2 value) override;
  void SetParameter(absl::string_view parameter_name, bool3 value) override;
  void SetParameter(absl::string_view parameter_name, bool4 value) override;
  void SetParameter(absl::string_view parameter_name, float value) override;
  void SetParameter(absl::string_view parameter_name, float2 value) override;
  void SetParameter(absl::string_view parameter_name, float3 value) override;
  void SetParameter(absl::string_view parameter_name, float4 value) override;
  void SetParameter(absl::string_view parameter_name, int value) override;
  void SetParameter(absl::string_view parameter_name, int2 value) override;
  void SetParameter(absl::string_view parameter_name, int3 value) override;
  void SetParameter(absl::string_view parameter_name, int4 value) override;
  void SetParameter(absl::string_view parameter_name, uint value) override;
  void SetParameter(absl::string_view parameter_name, uint2 value) override;
  void SetParameter(absl::string_view parameter_name, uint3 value) override;
  void SetParameter(absl::string_view parameter_name, uint4 value) override;
  void SetParameter(absl::string_view parameter_name, mat3f value) override;
  void SetParameter(absl::string_view parameter_name, mat4f value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const bool> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const bool2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const bool3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const bool4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const float> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const float2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const float3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const float4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const int> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const int2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const int3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const int4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const uint> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const uint2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const uint3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const uint4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const mat3f> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const mat4f> value) override;

  // Sets the value of a RGBA parameter in the underlying Filament structures.
  void SetParameter(absl::string_view parameter_name, filament::RgbaType type,
                    filament::math::float4 color) override;

  // Sets the value of a RGB parameter in the underlying Filament structures.
  void SetParameter(absl::string_view parameter_name, filament::RgbType type,
                    filament::math::float3 color) override;

  // Set a parameter in filament structures using texture.
  // Does not take ownership of texture.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  ABSL_DEPRECATED(
      "Use imp::BorrowedTexturePtr overload instead. See "
      "(broken link).")
  void SetParameter(
      absl::string_view parameter_name, const imp::Texture* texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  // Forwards the TexturePtr to the overload that takes an OwnedTexturePtr.
  //
  // This method is kept for backwards compatibility. Typically, TexturePtr can
  // be implicitly converted to OwnedTexturePtr. However, in this overload is
  // needed to disambiguate which overload of SetParameter is called.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  void SetParameter(
      absl::string_view parameter_name, TexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  // Set a parameter in filament structures using texture.
  //
  // Takes full ownership of texture.  The texture will be destroyed when this
  // material is.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  void SetParameter(
      absl::string_view parameter_name, OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  // Set a parameter in filament structures using texture.
  //
  // The OwnedTexturePtr that texture was borrowed from must not be destroyed
  // until after the material is either destroyed or SetParameter has been
  // called again to change to a different texture.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  void SetParameter(
      absl::string_view parameter_name, BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  // Returns true if parameter_name exists in the underlying Filament
  // structures.
  bool HasParameter(absl::string_view name) override;

  // Gets the name of the transform field associated for the given sampler
  // parameter. In the case where the parameter does not have a transform name
  // field, it will return an empty string.
  absl::string_view GetParameterTransformName(
      absl::string_view sampler_name) const override;

  // Returns the type of texture assignment for the given parameter name.
  HeldTextureType GetAssignedTextureType(
      absl::string_view parameter_name) override;

  // Returns a map of unowned filament textures used by the material.
  imp::StringMap<const filament::Texture*> GetUnownedFilamentTextures()
      const override;

  // Invokes the given function on each texture used by the material.
  void ForEachTexture(absl::FunctionRef<void(BorrowedTexturePtr)> fn,
                      SmallSourceLocation loc) override;

 protected:
  BorrowedMaterialPtr GetMaterialInternal(
      SmallSourceLocation loc) const override;

 private:
  // Information about the parameters and samplers in the material.
  struct ParameterInfo {
    std::vector<filament::Material::ParameterInfo> material_parameters;
    StringSet sampler_parameters;
    int max_available_samplers = 0;
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
  OwnedMaterialPtr material_;
  ParameterInfo parameter_info_;
  BorrowedTexturePtr placeholder_texture_;
  filament::TextureSampler placeholder_sampler_;

  // TODO (broken link) - Remove name_, parameters_, material_textures_, and
  // sampler_index_lookup_ once MaterialConfig is removed.
  std::vector<model::MaterialParameter> parameters_;
  TypedVector<model::MaterialTexture> material_textures_;
  StringMap<int> sampler_index_lookup_;

  // New system for storing texture data for easy retrieval, i.e. can get the
  // texture/sampler for a given texture channel name such as kBaseColorIndex.
  RobinMap<absl::string_view, TextureAndSampler> texture_lookup_;

  int next_available_assignable_sampler_index_ = 0;
  int samplers_uv_bitflags_ = 0;
  std::vector<mat3f> samplers_uv_matrices_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_IMPL_H_
