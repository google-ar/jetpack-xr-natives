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

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/asset_animator.h"
#include "apibindings/asset_loader.h"
#include "apibindings/impress_api_test_context.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/stereo_surface.h"
#include "core/material_library/generic_material_spec.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "core/view/utils/frame_time.h"

namespace imp {

void ImpressApiView::SetupImpressApiNative() {
  IMP_LOG(imp::INFO) << "Test ImpressApiView is setup";
}

void ImpressApiView::LoadGltfAsset(absl::string_view path,
                                   std::unique_ptr<AssetLoader> asset_loader) {
  ImpressApiTestContext& context = ImpressApiTestContext::Get();
  context.actual_gltf_path = std::string(path);

  

  if (asset_loader != nullptr) {
    if (!context.gltf_asset_loader_failure_message.empty()) {
      asset_loader->OnFailure(context.gltf_asset_loader_failure_message);
    } else {
      asset_loader->OnSuccess(context.gltf_asset_loader_success_token);
    }
  }
}

absl::StatusOr<int32_t> ImpressApiView::InstanceGltfModel(
    std::intptr_t gltf_token, bool enable_collider) {
  return absl::UnimplementedError(
      "ImpressApiView::InstanceGltfModel needs to be implemented for tests.");
}

absl::Status ImpressApiView::ReleaseGltfAsset(std::intptr_t gltf_token) {
  return absl::UnimplementedError(
      "ImpressApiView::ReleaseGltfAsset needs to be "
      "implemented for tests.");
}

void ImpressApiView::Setup() {}
void ImpressApiView::Update(const FrameTime& frame_time) {}

void ImpressApiView::LoadImageBasedLightingAsset(
    absl::string_view path, std::unique_ptr<AssetLoader> asset_loader) {
  IMP_LOG(imp::ERROR) << "ImpressApiView::LoadImageBasedLightingAsset needs to be "
                "implemented for tests.";
}

void ImpressApiView::LoadImageBasedLightingAsset(
    absl::Cord data, absl::string_view key,
    std::unique_ptr<AssetLoader> asset_loader) {
  IMP_LOG(imp::ERROR) << "ImpressApiView::LoadImageBasedLightingAsset needs to be "
                "implemented for tests.";
}

absl::Status ImpressApiView::ReleaseImageBasedLightingAsset(
    std::intptr_t ibl_token) {
  return absl::UnimplementedError(
      "ImpressApiView::ReleaseImageBasedLightingAsset needs to be "
      "implemented for tests.");
}

void ImpressApiView::LoadGltfAsset(absl::Cord data, absl::string_view key,
                                   std::unique_ptr<AssetLoader> asset_loader) {
  IMP_LOG(imp::ERROR) << "ImpressApiView::LoadGltfAsset needs to be implemented "
                "for tests.";
}

absl::Status ImpressApiView::SetGltfModelColliderEnabled(int32_t node,
                                                         bool enable_collider) {
  return absl::UnimplementedError(
      "ImpressApiView::SetGltfModelColliderEnabled needs to be "
      "implemented for tests.");
}

void ImpressApiView::AnimateGltfModel(
    int32_t node, absl::string_view animation_name, bool loop,
    std::unique_ptr<AssetAnimator> asset_animator) {
  IMP_LOG(imp::ERROR) << "ImpressApiView::AnimateGltfModel needs to be implemented for "
                "tests.";
}

absl::Status ImpressApiView::StopGltfModelAnimation(int32_t node) {
  return absl::UnimplementedError(
      "ImpressApiView::StopGltfModelAnimation needs to be "
      "implemented for tests.");
}

int32_t ImpressApiView::CreateImpressNode() {
  IMP_LOG(imp::ERROR)
      << "ImpressApiView::CreateImpressNode needs to be implemented for tests.";
  return -1;
}

absl::Status ImpressApiView::DestroyImpressNode(int32_t node) {
  return absl::UnimplementedError(
      "ImpressApiView::DestroyImpressNode needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetImpressNodeParent(int32_t child,
                                                  int32_t parent) {
  return absl::UnimplementedError(
      "ImpressApiView::SetImpressNodeParent needs to be "
      "implemented for tests.");
}

absl::StatusOr<int32_t> ImpressApiView::CreateStereoSurfaceEntity(
    MediaStereoMode stereo_mode, ContentSecurityLevel content_security_level,
    bool use_super_sampling) {
  IMP_LOG(imp::ERROR) << "ImpressApiView::CreateStereoSurfaceEntity needs to be "
                "implemented for tests.";
  return -1;
}

absl::Status ImpressApiView::SetStereoSurfaceEntityCanvasShape(
    int32_t node_id, StereoSurface::CanvasShape canvas_shape) {
  return absl::UnimplementedError(
      "ImpressApiView::SetStereoSurfaceEntityCanvasShape needs to be "
      "implemented for tests.");
}

absl::StatusOr<android::Surface*>
ImpressApiView::GetSurfaceFromStereoSurfaceEntity(int32_t node_id) {
  return absl::UnimplementedError(
      "ImpressApiView::GetSurfaceFromStereoSurfaceEntity needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetFeatherRadiusForStereoSurfaceEntity(
    int32_t node_id, const float2& feather_radius) {
  return absl::UnimplementedError(
      "ImpressApiView::SetFeatherRadiusForStereoSurfaceEntity needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetStereoModeForStereoSurfaceEntity(
    int32_t node_id, MediaStereoMode stereo_mode) {
  return absl::UnimplementedError(
      "ImpressApiView::SetStereoModeForStereoSurfaceEntity needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetPrimaryAlphaMaskForStereoSurfaceEntity(
    int32_t node_id, int64_t alpha_mask_token) {
  return absl::UnimplementedError(
      "ImpressApiView::SetPrimaryAlphaMaskForStereoSurfaceEntity needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetAuxiliaryAlphaMaskForStereoSurfaceEntity(
    int32_t node_id, int64_t alpha_mask_token) {
  return absl::UnimplementedError(
      "ImpressApiView::SetAuxiliaryAlphaMaskForStereoSurfaceEntity needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetContentColorMetadataForStereoSurfaceEntity(
    int32_t node_id, MediaColorSpace color_space) {
  return absl::UnimplementedError(
      "ImpressApiView::SetContentColorMetadataForStereoSurfaceEntity needs to "
      "be implemented for tests.");
}

void ImpressApiView::LoadTexture(absl::string_view path,
                                 std::unique_ptr<AssetLoader> asset_loader) {
  IMP_LOG(imp::ERROR)
      << "ImpressApiView::LoadTexture needs to be implemented for tests.";
}

absl::StatusOr<std::intptr_t> ImpressApiView::BorrowReflectionTexture() {
  return absl::UnimplementedError(
      "ImpressApiView::BorrowReflectionTexture needs to be "
      "implemented for tests.");
}

absl::StatusOr<std::intptr_t> ImpressApiView::GetReflectionTextureFromIbl(
    std::intptr_t ibl_token) {
  return absl::UnimplementedError(
      "ImpressApiView::GetReflectionTextureFromIbl needs to be "
      "implemented for tests.");
}

void ImpressApiView::CreateWaterMaterial(
    std::unique_ptr<AssetLoader> asset_loader, bool is_alpha_map_version) {
  IMP_LOG(imp::ERROR) << "ImpressApiView::CreateWaterMaterial needs to be implemented "
                "for tests.";
}

void ImpressApiView::DestroyNativeObject(std::intptr_t handle) {
  IMP_LOG(imp::ERROR) << "ImpressApiView::DestroyNativeObject needs to be implemented "
                "for tests.";
}

absl::Status ImpressApiView::SetReflectionMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t reflection_map,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetReflectionMapOnWaterMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetNormalMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t normal_map,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetNormalMapOnWaterMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetNormalTilingOnWaterMaterial(
    std::intptr_t water_material, float normal_tiling) {
  return absl::UnimplementedError(
      "ImpressApiView::SetNormalTilingOnWaterMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetNormalSpeedOnWaterMaterial(
    std::intptr_t water_material, float normal_speed) {
  return absl::UnimplementedError(
      "ImpressApiView::SetNormalSpeedOnWaterMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetAlphaStepMultiplierOnWaterMaterial(
    std::intptr_t water_material, float alpha_step_multiplier) {
  return absl::UnimplementedError(
      "ImpressApiView::SetAlphaStepMultiplierOnWaterMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetAlphaMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t alpha_map,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetAlphaMapOnWaterMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetNormalZOnWaterMaterial(
    std::intptr_t water_material, float normal_z) {
  return absl::UnimplementedError(
      "ImpressApiView::SetNormalZOnWaterMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetNormalBoundaryOnWaterMaterial(
    std::intptr_t water_material, float normal_boundary) {
  return absl::UnimplementedError(
      "ImpressApiView::SetNormalBoundaryOnWaterMaterial needs to be "
      "implemented for tests.");
}

void ImpressApiView::CreateGenericMaterial(
    std::unique_ptr<AssetLoader> asset_loader,
    imp::GenericMaterialSpec generic_material_spec) {
  IMP_LOG(imp::ERROR) << "ImpressApiView::CreateGenericMaterial needs to be implemented "
                "for tests.";
}

absl::Status ImpressApiView::SetBaseColorTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t base_color_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetBaseColorTextureOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetBaseColorUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "ImpressApiView::SetBaseColorUvTransformOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetBaseColorFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float4& factors) {
  return absl::UnimplementedError(
      "ImpressApiView::SetBaseColorFactorsOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetMetallicRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t metallic_roughness_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetMetallicRoughnessTextureOnGenericMaterial needs to "
      "be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetMetallicRoughnessUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "ImpressApiView::SetMetallicRoughnessUvTransformOnGenericMaterial needs "
      "to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetMetallicFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "ImpressApiView::SetMetallicFactorOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetRoughnessFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "ImpressApiView::SetRoughnessFactorOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetNormalTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t normal_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetNormalTextureOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetNormalUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "ImpressApiView::SetNormalUvTransformOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetNormalFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "ImpressApiView::SetNormalFactorOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetAmbientOcclusionTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t ambient_occlusion_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetAmbientOcclusionTextureOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetAmbientOcclusionUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "ImpressApiView::SetAmbientOcclusionUvTransformOnGenericMaterial needs "
      "to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetAmbientOcclusionFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "ImpressApiView::SetAmbientOcclusionFactorOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetEmissiveTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t emissive_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetEmissiveTextureOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetEmissiveUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "ImpressApiView::SetEmissiveUvTransformOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetEmissiveFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factors) {
  return absl::UnimplementedError(
      "ImpressApiView::SetEmissiveFactorsOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetClearcoatTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetClearcoatTextureOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetClearcoatNormalTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_normal_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetClearcoatNormalTextureOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetClearcoatRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t clearcoat_roughness_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetClearcoatRoughnessTextureOnGenericMaterial needs to "
      "be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetClearcoatFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factor) {
  return absl::UnimplementedError(
      "ImpressApiView::SetClearcoatFactorsOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetSheenColorTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t sheen_color_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetSheenColorTextureOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetSheenColorFactorsOnGenericMaterial(
    std::intptr_t generic_material, const float3& factors) {
  return absl::UnimplementedError(
      "ImpressApiView::SetSheenColorFactorsOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetSheenRoughnessTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t sheen_roughness_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetSheenRoughnessTextureOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetSheenRoughnessFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "ImpressApiView::SetSheenRoughnessFactorOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetTransmissionTextureOnGenericMaterial(
    std::intptr_t generic_material, std::intptr_t transmission_texture,
    std::optional<filament::TextureSampler> sampler) {
  return absl::UnimplementedError(
      "ImpressApiView::SetTransmissionTextureOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetTransmissionUvTransformOnGenericMaterial(
    std::intptr_t generic_material, const mat3f& uv_transform) {
  return absl::UnimplementedError(
      "ImpressApiView::SetTransmissionUvTransformOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetTransmissionFactorOnGenericMaterial(
    std::intptr_t generic_material, float factor) {
  return absl::UnimplementedError(
      "ImpressApiView::SetTransmissionFactorOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetIndexOfRefractionOnGenericMaterial(
    std::intptr_t generic_material, float index_of_refraction) {
  return absl::UnimplementedError(
      "ImpressApiView::SetIndexOfRefractionOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetAlphaCutoffOnGenericMaterial(
    std::intptr_t generic_material, float alpha_cutoff) {
  return absl::UnimplementedError(
      "ImpressApiView::SetAlphaCutoffOnGenericMaterial needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetMaterialOverride(int32_t node_id,
                                                 std::intptr_t material,
                                                 absl::string_view mesh_name) {
  return absl::UnimplementedError(
      "ImpressApiView::SetMaterialOverride needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::SetEnvironmentLight(std::intptr_t ibl_token) {
  return absl::UnimplementedError(
      "ImpressApiView::SetEnvironmentLight needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::ClearEnvironmentLight() {
  return absl::UnimplementedError(
      "ImpressApiView::ClearEnvironmentLight needs to be "
      "implemented for tests.");
}

absl::Status ImpressApiView::DisposeAllResources() {
  return absl::UnimplementedError(
      "ImpressApiView::DisposeAllResources needs to be "
      "implemented for tests.");
}

absl::StatusOr<BorrowedTexturePtr> ImpressApiView::BorrowTexture(
    std::intptr_t texture_handle) {
  return absl::UnimplementedError(
      "ImpressApiView::BorrowTexture needs to be "
      "implemented for tests.");
}

}  // namespace imp
