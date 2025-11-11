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

#include "apibindings/water_material_manager.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/bindings_material.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/texture_manager.h"
#include "core/async/future.h"
#include "core/common/hash.h"
#include "core/common/owned_ptr.h"
#include "core/common/small_source_location.h"
#include "core/common/type_traits.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "split_engine/materials/water_reflection_material.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace {

class WaterMaterialManagerImpl : public WaterMaterialManager {
 public:
  explicit WaterMaterialManagerImpl(ImpressApiView& view);
  ~WaterMaterialManagerImpl() override = default;

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

 private:
  // Helper to safely get a typed pointer to a water material from a handle.
  absl::StatusOr<android_xr::WaterReflectionMaterial*> GetWaterMaterial(
      std::intptr_t material_handle);

  // Helper to reduce boilerplate when setting a texture parameter.
  template <typename SetterFn>
  absl::Status SetTextureParameter(
      std::intptr_t water_material, std::intptr_t texture,
      std::optional<filament::TextureSampler> sampler, SetterFn setter_fn);

  ImpressApiView& view_;
};

}  // namespace

WaterMaterialManagerImpl::WaterMaterialManagerImpl(ImpressApiView& view)
    : view_(view) {}

void WaterMaterialManagerImpl::CreateWaterMaterial(
    std::unique_ptr<BaseAssetLoader> asset_loader, bool is_alpha_map_version) {
  android_xr::WaterReflectionMaterial::Create(view_, is_alpha_map_version)
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<
                    std::unique_ptr<android_xr::WaterReflectionMaterial>>
                    material) mutable {
        if (material.ok() && *material != nullptr) {
          OwnedPtr<android_xr::WaterReflectionMaterial> owned_material_ptr(
              *std::move(material));
          std::intptr_t material_token = view_.ToJava(new BindingsMaterial(
              owned_material_ptr->GetMaterial(SmallSourceLocation::Current()),
              type_traits::kTypeHash<android_xr::WaterReflectionMaterial>));
          view_.GetBindingsMaterialMap().emplace(material_token,
                                                 std::move(owned_material_ptr));
          asset_loader->OnSuccess(material_token);
        } else {
          asset_loader->OnFailure(absl::StrFormat(
              "Failed to create the built-in water material: %s.",
              material.status().message()));
        }
      })
      .KeptBy(&view_);
}

absl::StatusOr<android_xr::WaterReflectionMaterial*>
WaterMaterialManagerImpl::GetWaterMaterial(std::intptr_t material_handle) {
  BindingsMaterial* bindings_material =
      view_.FromJava<BindingsMaterial>(material_handle);
  if (!bindings_material) {
    return absl::InvalidArgumentError("Provided material handle is not valid.");
  }

  HashValue material_type_hash = bindings_material->GetTypeHash();
  HashValue expected_type_hash =
      type_traits::kTypeHash<android_xr::WaterReflectionMaterial>;
  if (material_type_hash != expected_type_hash) {
    return absl::InvalidArgumentError(
        "Provided material handle is not of the correct type.");
  }

  split_engine::SplitEngineMaterial* base_material =
      &(*view_.GetBindingsMaterialMap().at(material_handle));
  android_xr::WaterReflectionMaterial* derived_material =
      static_cast<android_xr::WaterReflectionMaterial*>(base_material);
  if (!derived_material) {
    return absl::InternalError(
        "Material type hash matched, but static_cast failed.");
  }

  return derived_material;
}

template <typename SetterFn>
absl::Status WaterMaterialManagerImpl::SetTextureParameter(
    std::intptr_t water_material, std::intptr_t texture,
    std::optional<filament::TextureSampler> sampler, SetterFn setter_fn) {
  MP_ASSIGN_OR_RETURN(android_xr::WaterReflectionMaterial * material_ptr,
                   GetWaterMaterial(water_material));
  MP_ASSIGN_OR_RETURN(BorrowedTexturePtr borrowed_texture,
                   view_.GetTextureManager().BorrowTexture(texture));

  setter_fn(material_ptr, borrowed_texture, sampler);
  return absl::OkStatus();
}

absl::Status WaterMaterialManagerImpl::SetReflectionMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t reflection_map,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      water_material, reflection_map, sampler,
      [](android_xr::WaterReflectionMaterial* material,
         BorrowedTexturePtr borrowed_texture,
         std::optional<filament::TextureSampler> maybe_sampler) {
        material->SetReflectionCube(borrowed_texture, maybe_sampler);
      });
}

absl::Status WaterMaterialManagerImpl::SetNormalMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t normal_map,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      water_material, normal_map, sampler,
      [](android_xr::WaterReflectionMaterial* material,
         BorrowedTexturePtr borrowed_texture,
         std::optional<filament::TextureSampler> maybe_sampler) {
        material->SetNormalMap(borrowed_texture, maybe_sampler);
      });
}

absl::Status WaterMaterialManagerImpl::SetNormalTilingOnWaterMaterial(
    std::intptr_t water_material, float normal_tiling) {
  MP_ASSIGN_OR_RETURN(android_xr::WaterReflectionMaterial * material,
                   GetWaterMaterial(water_material));
  material->SetNormalTiling(normal_tiling);
  return absl::OkStatus();
}

absl::Status WaterMaterialManagerImpl::SetNormalSpeedOnWaterMaterial(
    std::intptr_t water_material, float normal_speed) {
  MP_ASSIGN_OR_RETURN(android_xr::WaterReflectionMaterial * material,
                   GetWaterMaterial(water_material));
  material->SetNormalSpeed(normal_speed);
  return absl::OkStatus();
}

absl::Status WaterMaterialManagerImpl::SetAlphaStepMultiplierOnWaterMaterial(
    std::intptr_t water_material, float alpha_step_multiplier) {
  MP_ASSIGN_OR_RETURN(android_xr::WaterReflectionMaterial * material,
                   GetWaterMaterial(water_material));
  material->SetAlphaStepMultiplier(alpha_step_multiplier);
  return absl::OkStatus();
}

absl::Status WaterMaterialManagerImpl::SetAlphaMapOnWaterMaterial(
    std::intptr_t water_material, std::intptr_t alpha_map,
    std::optional<filament::TextureSampler> sampler) {
  return SetTextureParameter(
      water_material, alpha_map, sampler,
      [](android_xr::WaterReflectionMaterial* material,
         BorrowedTexturePtr borrowed_texture,
         std::optional<filament::TextureSampler> maybe_sampler) {
        material->SetAlphaMap(borrowed_texture, maybe_sampler);
      });
}

absl::Status WaterMaterialManagerImpl::SetNormalZOnWaterMaterial(
    std::intptr_t water_material, float normal_z) {
  MP_ASSIGN_OR_RETURN(android_xr::WaterReflectionMaterial * material,
                   GetWaterMaterial(water_material));
  material->SetNormalZ(normal_z);
  return absl::OkStatus();
}

absl::Status WaterMaterialManagerImpl::SetNormalBoundaryOnWaterMaterial(
    std::intptr_t water_material, float normal_boundary) {
  MP_ASSIGN_OR_RETURN(android_xr::WaterReflectionMaterial * material,
                   GetWaterMaterial(water_material));
  material->SetNormalBoundary(normal_boundary);
  return absl::OkStatus();
}

std::unique_ptr<WaterMaterialManager> CreateWaterMaterialManager(
    ImpressApiView& view) {
  return std::make_unique<WaterMaterialManagerImpl>(view);
}

}  // namespace imp
