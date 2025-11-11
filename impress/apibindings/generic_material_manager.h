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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_GENERIC_MATERIAL_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_GENERIC_MATERIAL_MANAGER_H_

#include <cstdint>
#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/base_asset_loader.h"
#include "core/material_library/generic_material_spec.h"
#include "core/math/math.h"

namespace imp {

// Manages SplitEngineGenericMaterial instances for the Impress API for the
// Jetpack XR Scene.
class GenericMaterialManager {
 public:
  virtual ~GenericMaterialManager() = default;

  // Creates a new generic material using a given spec, and resolves the asset
  // loader when it is ready.
  virtual void CreateGenericMaterial(
      std::unique_ptr<BaseAssetLoader> asset_loader,
      imp::GenericMaterialSpec generic_material_spec) = 0;

  // Sets the base color texture for the generic material.
  virtual absl::Status SetBaseColorTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t base_color_texture,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the UV transformation matrix for the base color texture.
  virtual absl::Status SetBaseColorUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) = 0;

  // Sets the base color factors for the generic material.
  virtual absl::Status SetBaseColorFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float4& factors) = 0;

  // Sets the metallic-roughness texture for the generic material.
  virtual absl::Status SetMetallicRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t metallic_roughness_texture,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the UV transformation matrix for the metallic-roughness texture.
  virtual absl::Status SetMetallicRoughnessUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) = 0;

  // Sets the metallic factor for the generic material.
  virtual absl::Status SetMetallicFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) = 0;

  // Sets the roughness factor for the generic material.
  virtual absl::Status SetRoughnessFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) = 0;

  // Sets the normal map texture for the generic material.
  virtual absl::Status SetNormalTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t normal_texture,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the UV transformation matrix for the normal map texture.
  virtual absl::Status SetNormalUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) = 0;

  // Sets the factor of the normal map effect.
  virtual absl::Status SetNormalFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) = 0;

  // Sets the ambient occlusion texture for the generic material.
  virtual absl::Status SetAmbientOcclusionTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t ambient_occlusion_texture,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the UV transformation matrix for the ambient occlusion texture.
  virtual absl::Status SetAmbientOcclusionUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) = 0;

  // Sets the factor of the ambient occlusion effect.
  virtual absl::Status SetAmbientOcclusionFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) = 0;

  // Sets the emissive texture for the generic material.
  virtual absl::Status SetEmissiveTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t emissive_texture,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the UV transformation matrix for the emissive texture.
  virtual absl::Status SetEmissiveUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) = 0;

  // Sets the emissive color factors for the generic material.
  virtual absl::Status SetEmissiveFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factors) = 0;

  // Sets the clearcoat texture for the generic material.
  virtual absl::Status SetClearcoatTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_texture,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the clearcoat normal texture for the generic material.
  virtual absl::Status SetClearcoatNormalTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_normal_texture,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the clearcoat roughness texture for the generic material.
  virtual absl::Status SetClearcoatRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_roughness_texture,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the clearcoat factors for the generic material.
  virtual absl::Status SetClearcoatFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factor) = 0;

  // Sets the sheen color texture for the generic material.
  virtual absl::Status SetSheenColorTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t sheen_color_texture,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the sheen color factors for the generic material.
  virtual absl::Status SetSheenColorFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factors) = 0;

  // Sets the sheen roughness texture for the generic material.
  virtual absl::Status SetSheenRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t sheen_roughness_texture,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the sheen roughness factor for the generic material.
  virtual absl::Status SetSheenRoughnessFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) = 0;

  // Sets the transmission texture for the generic material.
  virtual absl::Status SetTransmissionTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t transmission_texture,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the UV transformation matrix for the transmission texture.
  virtual absl::Status SetTransmissionUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform) = 0;

  // Sets the transmission factor for the generic material.
  virtual absl::Status SetTransmissionFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor) = 0;

  // Sets the index of refraction for the generic material.
  virtual absl::Status SetIndexOfRefractionOnGenericMaterial(
      std::intptr_t generic_material, float index_of_refraction) = 0;

  // Sets the alpha cutoff for the generic material.
  virtual absl::Status SetAlphaCutoffOnGenericMaterial(
      std::intptr_t generic_material, float alpha_cutoff) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_GENERIC_MATERIAL_MANAGER_H_
