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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_WATER_MATERIAL_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_WATER_MATERIAL_MANAGER_H_

#include <cstdint>
#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/water_material_manager.h"

namespace imp {

// Inherits from the real WaterMaterialManager for testing purposes.
class TestWaterMaterialManager : public WaterMaterialManager {
 public:
  explicit TestWaterMaterialManager(ImpressApiView& view);
  ~TestWaterMaterialManager() override = default;

  void CreateWaterMaterial(std::unique_ptr<BaseAssetLoader> asset_loader,
                           bool is_alpha_map_version) override;
  absl::Status SetReflectionMapOnWaterMaterial(
      std::intptr_t water_material, std::intptr_t reflection_map,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetNormalMapOnWaterMaterial(
      std::intptr_t water_material, std::intptr_t normal_map,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetNormalTilingOnWaterMaterial(std::intptr_t water_material,
                                              float normal_tiling) override;
  absl::Status SetNormalSpeedOnWaterMaterial(std::intptr_t water_material,
                                             float normal_speed) override;
  absl::Status SetAlphaStepMultiplierOnWaterMaterial(
      std::intptr_t water_material, float alpha_step_multiplier) override;
  absl::Status SetAlphaMapOnWaterMaterial(
      std::intptr_t water_material, std::intptr_t alpha_map,
      std::optional<filament::TextureSampler> sampler) override;
  absl::Status SetNormalZOnWaterMaterial(std::intptr_t water_material,
                                         float normal_z) override;
  absl::Status SetNormalBoundaryOnWaterMaterial(std::intptr_t water_material,
                                                float normal_boundary) override;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_WATER_MATERIAL_MANAGER_H_
