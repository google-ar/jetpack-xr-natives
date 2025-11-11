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

#include "apibindings/skybox_manager.h"

#include <cstdint>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "apibindings/asset_ptr_map.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"
#include "core/assets/asset_ptr.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/math/math.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/framework/lighting/light_manager.h"

namespace imp {

namespace {

class SkyboxManagerImpl : public SkyboxManager {
 public:
  explicit SkyboxManagerImpl(ImpressApiView& view);
  ~SkyboxManagerImpl() override = default;

  void LoadImageBasedLightingAsset(
      absl::string_view path,
      std::unique_ptr<BaseAssetLoader> asset_loader) override;
  void LoadImageBasedLightingAsset(
      absl::Cord data, absl::string_view key,
      std::unique_ptr<BaseAssetLoader> asset_loader) override;
  absl::Status ReleaseImageBasedLightingAsset(std::intptr_t ibl_token) override;
  absl::Status SetEnvironmentLight(std::intptr_t ibl_token) override;
  absl::Status ClearEnvironmentLight() override;

 private:
  ImpressApiView& view_;
};

}  // namespace

SkyboxManagerImpl::SkyboxManagerImpl(ImpressApiView& view) : view_(view) {}

void SkyboxManagerImpl::LoadImageBasedLightingAsset(
    absl::string_view path, std::unique_ptr<BaseAssetLoader> asset_loader) {
  view_.GetAssetPtrMap().LoadImageBasedLightingAsset(path,
                                                     std::move(asset_loader));
}

void SkyboxManagerImpl::LoadImageBasedLightingAsset(
    absl::Cord data, absl::string_view key,
    std::unique_ptr<BaseAssetLoader> asset_loader) {
  view_.GetAssetPtrMap().LoadImageBasedLightingAsset(data, key,
                                                     std::move(asset_loader));
}

absl::Status SkyboxManagerImpl::ReleaseImageBasedLightingAsset(
    std::intptr_t ibl_token) {
  return view_.GetAssetPtrMap().ReleaseImageBasedLightingAsset(ibl_token);
}

absl::Status SkyboxManagerImpl::SetEnvironmentLight(std::intptr_t ibl_token) {
  absl::StatusOr<AssetPtr<ImageBasedLightingAsset>> ibl_asset_ptr =
      view_.GetAssetPtrMap().GetStoredIblAsset(ibl_token);
  if (!ibl_asset_ptr.ok()) {
    return absl::NotFoundError(absl::StrFormat(
        "IBL asset is not cached: %s.", ibl_asset_ptr.status().message()));
  }
  split_engine::SplitEngineSerializer* serializer =
      view_.GetSplitEngineSerializer();
  if (serializer == nullptr) {
    return absl::InternalError("SplitEngineSerializer is not available.");
  }
  serializer->SetPreferredEnvironmentIblAsset(
      *ibl_asset_ptr.value()->BorrowReflectionTexture()->GetTexture(),
      LightManager::kDefaultEnvironmentLightIntensity, kOne3);
  view_.GetLightManager().SetEnvironmentLight(
      view_.GetEnvironmentLightFactory().CreateEnvironmentLight(
          ibl_asset_ptr.value(),
          LightManager::kDefaultEnvironmentLightIntensity));
  return absl::OkStatus();
}

absl::Status SkyboxManagerImpl::ClearEnvironmentLight() {
  split_engine::SplitEngineSerializer* serializer =
      view_.GetSplitEngineSerializer();
  if (serializer == nullptr) {
    return absl::InternalError("SplitEngineSerializer is not available.");
  }
  serializer->ClearPreferredEnvironmentIblAsset();
  return absl::OkStatus();
}

std::unique_ptr<SkyboxManager> CreateSkyboxManager(ImpressApiView& view) {
  return std::make_unique<SkyboxManagerImpl>(view);
}

}  // namespace imp
