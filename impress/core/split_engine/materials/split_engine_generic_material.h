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
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
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
#include "core/split_engine/materials/split_engine_material.h"
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
class SplitEngineGenericMaterial : public SplitEngineMaterial,
                                   public GenericMaterial {
 public:
  // Creates a generic material on the remote renderer.
  // Please note that schema::GenericMaterialDepthClearMaterial is NOT
  // supported in Split Engine.
  static Future<std::unique_ptr<SplitEngineGenericMaterial>> Create(
      BaseView& view, const GenericMaterialSpec& spec);

  ~SplitEngineGenericMaterial() override;

  GenericMaterialPtr Duplicate() const override;

  absl::Status AssignTexturesAndParams(
      const GenericMaterialParameters& generic_material_parameters,
      const TextureBorrower& texture_borrower) override;

  absl::string_view GetName() const override;
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

  TextureAndSampler GetTransmissionTexture() const override;
  absl::Status SetTransmissionUvTransform(const mat3f& uv_transform) override;
  void SetTransmissionFactor(float factor) override;
  void SetIndexOfRefraction(float index_of_refraction) override;

  std::optional<TextureAndSampler> GetFeatureIdTexture(
      int index) const override;

  void SetAlphaCutoff(float alpha_cutoff) override;
  float GetAlphaCutoff() const override;

 protected:
  BorrowedMaterialPtr GetMaterialInternal(
      SmallSourceLocation loc) const override;

  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      BuiltInTextureParameterCreator& texture_parameter_creator) const override;

 private:
  SplitEngineGenericMaterial(BaseView& view,
                             OwnedMaterialPtr placeholder_material);
  // Gets the texture and sampler for the given texture parameter.
  TextureAndSampler GetTextureAndSampler(
      const GenericMaterialTextureParameter& texture) const;
  TextureAndSampler GetPlaceholderTextureAndSampler() const;

  BaseView& view_;
  GenericMaterialParameters generic_material_parameters_;
  BorrowedTexturePtr placeholder_texture_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_GENERIC_MATERIAL_FACTORY_H_
