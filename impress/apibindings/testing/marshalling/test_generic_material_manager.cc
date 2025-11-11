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

#include "apibindings/testing/marshalling/test_generic_material_manager.h"

#include <cstdint>
#include <memory>
#include <optional>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"
#include "core/material_library/generic_material_spec.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

namespace imp {

TestGenericMaterialManager::TestGenericMaterialManager(ImpressApiView& view) {}

void TestGenericMaterialManager::CreateGenericMaterial(
    std::unique_ptr<BaseAssetLoader> asset_loader,
    GenericMaterialSpec generic_material_spec) {
  IMP_LOG(imp::FATAL) << "TestGenericMaterialManager::CreateGenericMaterial "
                "unimplemented";
}
absl::Status TestGenericMaterialManager::SetBaseColorTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t base_color_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetBaseColorTextureOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetBaseColorUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetBaseColorUvTransformOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetBaseColorFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float4& factors) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetBaseColorFactorsOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetMetallicRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t metallic_roughness_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::"
      "SetMetallicRoughnessTextureOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetMetallicRoughnessUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::"
      "SetMetallicRoughnessUvTransformOnGenericMaterial unimplemented");
}
absl::Status TestGenericMaterialManager::SetMetallicFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetMetallicFactorOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetRoughnessFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetRoughnessFactorOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetNormalTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t normal_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetNormalTextureOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetNormalUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetNormalUvTransformOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetNormalFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetNormalFactorOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetAmbientOcclusionTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t ambient_occlusion_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetAmbientOcclusionTextureOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetAmbientOcclusionUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::"
      "SetAmbientOcclusionUvTransformOnGenericMaterial unimplemented");
}
absl::Status
TestGenericMaterialManager::SetAmbientOcclusionFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetAmbientOcclusionFactorOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetEmissiveTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t emissive_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetEmissiveTextureOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetEmissiveUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetEmissiveUvTransformOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetEmissiveFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factors) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetEmissiveFactorsOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetClearcoatTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetClearcoatTextureOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetClearcoatNormalTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_normal_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetClearcoatNormalTextureOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetClearcoatRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_roughness_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::"
      "SetClearcoatRoughnessTextureOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetClearcoatFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factor) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetClearcoatFactorsOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetSheenColorTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t sheen_color_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetSheenColorTextureOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetSheenColorFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factors) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetSheenColorFactorsOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetSheenRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t sheen_roughness_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetSheenRoughnessTextureOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetSheenRoughnessFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetSheenRoughnessFactorOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetTransmissionTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t transmission_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetTransmissionTextureOnGenericMaterial "
      "unimplemented");
}
absl::Status
TestGenericMaterialManager::SetTransmissionUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::"
      "SetTransmissionUvTransformOnGenericMaterial unimplemented");
}
absl::Status TestGenericMaterialManager::SetTransmissionFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetTransmissionFactorOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetIndexOfRefractionOnGenericMaterial(
    std::intptr_t generic_material, float index_of_refraction) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetIndexOfRefractionOnGenericMaterial "
      "unimplemented");
}
absl::Status TestGenericMaterialManager::SetAlphaCutoffOnGenericMaterial(
    std::intptr_t generic_material, float alpha_cutoff) {
  return absl::UnimplementedError(
      "TestGenericMaterialManager::SetAlphaCutoffOnGenericMaterial "
      "unimplemented");
}

}  // namespace imp
