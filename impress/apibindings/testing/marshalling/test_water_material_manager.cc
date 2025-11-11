// Copyright 2025 Google LLC
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

#include "apibindings/testing/marshalling/test_water_material_manager.h"

#include <cstdint>
#include <memory>
#include <optional>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"

namespace imp {

TestWaterMaterialManager::TestWaterMaterialManager(ImpressApiView& view) {}

void TestWaterMaterialManager::CreateWaterMaterial(
    std::unique_ptr<BaseAssetLoader> asset_loader, bool is_alpha_map_version) {
  IMP_LOG(imp::FATAL) << "TestWaterMaterialManager::CreateWaterMaterial unimplemented";
}

absl::Status TestWaterMaterialManager::SetReflectionMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t reflection_map,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestWaterMaterialManager::SetReflectionMapOnWaterMaterial "
      "unimplemented");
}

absl::Status TestWaterMaterialManager::SetNormalMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t normal_map,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestWaterMaterialManager::SetNormalMapOnWaterMaterial unimplemented");
}

absl::Status TestWaterMaterialManager::SetNormalTilingOnWaterMaterial(
    std::intptr_t water_material, float normal_tiling) {
  return absl::UnimplementedError(
      "TestWaterMaterialManager::SetNormalTilingOnWaterMaterial "
      "unimplemented");
}

absl::Status TestWaterMaterialManager::SetNormalSpeedOnWaterMaterial(
    std::intptr_t water_material, float normal_speed) {
  return absl::UnimplementedError(
      "TestWaterMaterialManager::SetNormalSpeedOnWaterMaterial unimplemented");
}

absl::Status TestWaterMaterialManager::SetAlphaStepMultiplierOnWaterMaterial(
    std::intptr_t water_material, float alpha_step_multiplier) {
  return absl::UnimplementedError(
      "TestWaterMaterialManager::SetAlphaStepMultiplierOnWaterMaterial "
      "unimplemented");
}

absl::Status TestWaterMaterialManager::SetAlphaMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t alpha_map,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestWaterMaterialManager::SetAlphaMapOnWaterMaterial unimplemented");
}

absl::Status TestWaterMaterialManager::SetNormalZOnWaterMaterial(
    std::intptr_t water_material, float normal_z) {
  return absl::UnimplementedError(
      "TestWaterMaterialManager::SetNormalZOnWaterMaterial unimplemented");
}

absl::Status TestWaterMaterialManager::SetNormalBoundaryOnWaterMaterial(
    std::intptr_t water_material, float normal_boundary) {
  return absl::UnimplementedError(
      "TestWaterMaterialManager::SetNormalBoundaryOnWaterMaterial "
      "unimplemented");
}

}  // namespace imp
