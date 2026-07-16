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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_GENERIC_MATERIAL_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_GENERIC_MATERIAL_FACTORY_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/container/flat_hash_map.h"
#include "absl/functional/function_ref.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/common/typed_vector.h"
#include "core/material_library/generic_material.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/model/entity_data.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"

namespace imp::model {
struct MaterialTexture;
}  // namespace imp::model

namespace imp::split_engine {

// This version of GenericMaterial is used instead of GenericMaterialImpl when
// running in Split Engine mode. It is a thin wrapper around the real generic
// material that is created on the remote renderer. It sends a request across
// the split engine bridge to create the material and then sends updates to
// set the parameters.
class SplitEngineGenericMaterial : public SplitEngineBuiltinMaterial,
                                   public GenericMaterial {
 public:
  // Creates a generic material on the remote renderer.
  // Please note that schema::GenericMaterialDepthClearMaterial is NOT
  // supported in Split Engine.
  static Future<std::unique_ptr<SplitEngineGenericMaterial>> Create(
      BaseView& view, const GenericMaterialSpec& spec);

  ~SplitEngineGenericMaterial() override;

  const std::string& GetName() const override;

  void SetName(absl::string_view name) override;

  GenericMaterialPtr Duplicate() const override;

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
  // imp::split_engine::SplitEngineGenericMaterial::Create(...)
  //     .Then([](std::unique_ptr<SplitEngineGenericMaterial> material) {
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

  TextureAndSampler GetThicknessTexture() const override;
  void SetThicknessFactor(float factor) override;
  void SetAttenuationDistance(float distance) override;
  void SetAttenuationColor(const float3& color) override;

  TextureAndSampler GetTransmissionTexture() const override;
  absl::Status SetTransmissionUvTransform(const mat3f& uv_transform) override;
  void SetTransmissionFactor(float factor) override;
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
  void ForEachTexture(
      absl::FunctionRef<void(BorrowedTexturePtr)> fn,
      SmallSourceLocation loc = SmallSourceLocation::Current()) override;

 protected:
  BorrowedMaterialPtr GetMaterialInternal(
      SmallSourceLocation loc) const override;

  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      BuiltInTextureParameterCreator& texture_parameter_creator) const override;

  // Generic material always disregards local mode.
  bool IsAlwaysRemote() const override { return true; }

 private:
  SplitEngineGenericMaterial(BaseView& view,
                             OwnedMaterialPtr placeholder_material);
  // Gets the texture and sampler for the given texture parameter.
  TextureAndSampler GetTextureAndSampler(
      const GenericMaterialTextureParameter& texture) const;
  TextureAndSampler GetPlaceholderTextureAndSampler() const;
  void RewriteTextureId(
      absl::string_view parameter_name,
      std::optional<GenericMaterialTextureParameter>& texture_parameter,
      const TextureBorrower& texture_borrower);
  GenericMaterialParameters RewriteTextureIds(
      const GenericMaterialParameters& parameters,
      const TextureBorrower& texture_borrower);

  std::string name_;
  GenericMaterialParameters generic_material_parameters_;
  BorrowedTexturePtr placeholder_texture_;
  absl::flat_hash_map<absl::string_view, BorrowedTexturePtr> borrowed_textures_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_GENERIC_MATERIAL_FACTORY_H_
