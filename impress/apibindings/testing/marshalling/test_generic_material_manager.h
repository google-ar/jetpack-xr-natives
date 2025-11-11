/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_GENERIC_MATERIAL_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_GENERIC_MATERIAL_MANAGER_H_

#include <cstdint>
#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/generic_material_manager.h"
#include "apibindings/impress_api_view.h"
#include "core/material_library/generic_material_spec.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

namespace imp {

// Inherits from the real GenericMaterialManager for testing purposes.
class TestGenericMaterialManager : public GenericMaterialManager {
 public:
  explicit TestGenericMaterialManager(ImpressApiView& view);
  ~TestGenericMaterialManager() override = default;

  void CreateGenericMaterial(
      std::unique_ptr<BaseAssetLoader> asset_loader,
      GenericMaterialSpec generic_material_spec) override;
  absl::Status SetBaseColorTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t base_color_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetBaseColorUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetBaseColorFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float4& factors) override;
  absl::Status SetMetallicRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t metallic_roughness_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetMetallicRoughnessUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetMetallicFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) override;
  absl::Status SetRoughnessFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) override;
  absl::Status SetNormalTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t normal_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetNormalUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetNormalFactorOnGenericMaterial(std::intptr_t generic_material,
                                                float factor) override;
  absl::Status SetAmbientOcclusionTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t ambient_occlusion_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetAmbientOcclusionUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetAmbientOcclusionFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) override;
  absl::Status SetEmissiveTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t emissive_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetEmissiveUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetEmissiveFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factors) override;
  absl::Status SetClearcoatTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetClearcoatNormalTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_normal_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetClearcoatRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_roughness_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetClearcoatFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factor) override;
  absl::Status SetSheenColorTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t sheen_color_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetSheenColorFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factors) override;
  absl::Status SetSheenRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t sheen_roughness_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetSheenRoughnessFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) override;
  absl::Status SetTransmissionTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t transmission_texture,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetTransmissionUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) override;
  absl::Status SetTransmissionFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) override;
  absl::Status SetIndexOfRefractionOnGenericMaterial(
      std::intptr_t generic_material, float index_of_refraction) override;
  absl::Status SetAlphaCutoffOnGenericMaterial(std::intptr_t generic_material,
                                               float alpha_cutoff) override;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_GENERIC_MATERIAL_MANAGER_H_
