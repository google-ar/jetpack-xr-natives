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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_WATER_MATERIAL_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_WATER_MATERIAL_MANAGER_H_

#include <cstdint>
#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/base_asset_loader.h"

namespace imp {

// Manages WaterReflectionMaterial instances for the Impress API  for the
// Jetpack XR Scene.
class WaterMaterialManager {
 public:
  virtual ~WaterMaterialManager() = default;

  // Creates a new regular or alpha map version of the water material, and
  // resolves the asset loader when it is ready.
  virtual void CreateWaterMaterial(
      std::unique_ptr<BaseAssetLoader> asset_loader,
      bool is_alpha_map_version) = 0;

  // Sets the reflection map for the water material.
  virtual absl::Status SetReflectionMapOnWaterMaterial(
      std::intptr_t water_material, std::intptr_t reflection_map,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the normal map for the water material.
  virtual absl::Status SetNormalMapOnWaterMaterial(
      std::intptr_t water_material, std::intptr_t normal_map,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the normal tiling for the water material.
  virtual absl::Status SetNormalTilingOnWaterMaterial(
      std::intptr_t water_material, float normal_tiling) = 0;

  // Sets the normal speed for the water material.
  virtual absl::Status SetNormalSpeedOnWaterMaterial(
      std::intptr_t water_material, float normal_speed) = 0;

  // Sets the alpha step multiplier for the water material.
  virtual absl::Status SetAlphaStepMultiplierOnWaterMaterial(
      std::intptr_t water_material, float alpha_step_multiplier) = 0;

  // Sets the alpha map for the water material.
  virtual absl::Status SetAlphaMapOnWaterMaterial(
      std::intptr_t water_material, std::intptr_t alpha_map,
      std::optional<filament::TextureSampler> sampler) = 0;

  // Sets the normal z for the water material.
  virtual absl::Status SetNormalZOnWaterMaterial(std::intptr_t water_material,
                                                 float normal_z) = 0;

  // Sets the normal boundary for the water material.
  virtual absl::Status SetNormalBoundaryOnWaterMaterial(
      std::intptr_t water_material, float normal_boundary) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_WATER_MATERIAL_MANAGER_H_
