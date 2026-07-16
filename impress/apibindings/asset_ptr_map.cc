// Copyright 2024 Google LLC
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

#include "apibindings/asset_ptr_map.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_loader.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/gltf/gltf_asset.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_state.proto.imp.h"
#include "core/view/utils/asset.h"
#include "re2/re2.h"

namespace imp {

AssetPtrMap::AssetPtrMap(BaseView& view) : view_(view) {}

void AssetPtrMap::LoadImageBasedLightingAsset(
    absl::string_view path, std::unique_ptr<BaseAssetLoader> asset_loader) {
  view_.GetAssetManager()
      .LoadAsset<ImageBasedLightingAsset>(GetAssetString(path))
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<AssetPtr<ImageBasedLightingAsset>>
                    asset_ptr) mutable {
        HandleAssetLoadingResult(asset_ptr, std::move(asset_loader),
                                 ibl_asset_map_);
      })
      .KeptBy(this);
}

void AssetPtrMap::LoadImageBasedLightingAsset(
    absl::Cord data, absl::string_view key,
    std::unique_ptr<BaseAssetLoader> asset_loader) {
  view_.GetAssetManager()
      .LoadAsset<ImageBasedLightingAsset>(data, key)
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<AssetPtr<ImageBasedLightingAsset>>
                    asset_ptr) mutable {
        HandleAssetLoadingResult(asset_ptr, std::move(asset_loader),
                                 ibl_asset_map_);
      })
      .KeptBy(this);
  ;
}

void AssetPtrMap::LoadImageBasedLightingAsset(
    imp::AssetDefinition asset_definition,
    std::unique_ptr<BaseAssetLoader> asset_loader) {
  view_.GetAssetManager()
      .LoadAsset<ImageBasedLightingAsset>(asset_definition)
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<AssetPtr<ImageBasedLightingAsset>>
                    asset_ptr) mutable {
        HandleAssetLoadingResult(asset_ptr, std::move(asset_loader),
                                 ibl_asset_map_);
      })
      .KeptBy(this);
}

absl::Status AssetPtrMap::ReleaseImageBasedLightingAsset(
    std::intptr_t ibl_token) {
  auto asset_ptr = ibl_asset_map_.find(ibl_token);
  if (asset_ptr == ibl_asset_map_.end()) {
    return absl::NotFoundError(absl::StrFormat(
        "Image based lighting asset with token %d is not cached.", ibl_token));
  }
  --asset_ptr->second.ref_count;
  if (asset_ptr->second.ref_count <= 0) {
    ibl_asset_map_.erase(asset_ptr);
  }
  return absl::OkStatus();
}

void AssetPtrMap::LoadGltfAsset(absl::string_view path,
                                std::unique_ptr<BaseAssetLoader> asset_loader) {
  auto load_options = GltfAsset::LoadOptions(
      {.collider_mode =
           imp::GltfState::ColliderMode::GLTF_COLLIDER_TRIANGLES_PER_MESH});
  view_.GetAssetManager()
      .LoadGltfAsset(GetAssetString(path), load_options)
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<AssetPtr<GltfAsset>> asset_ptr) mutable {
        HandleAssetLoadingResult(asset_ptr, std::move(asset_loader),
                                 gltf_asset_map_);
      })
      .KeptBy(this);
}

void AssetPtrMap::LoadGltfAsset(absl::Cord data, absl::string_view key,
                                std::unique_ptr<BaseAssetLoader> asset_loader) {
  auto load_options = GltfAsset::LoadOptions(
      {.collider_mode =
           imp::GltfState::ColliderMode::GLTF_COLLIDER_TRIANGLES_PER_MESH});
  view_.GetAssetManager()
      .LoadGltfAsset(data, key, load_options)
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<AssetPtr<GltfAsset>> asset_ptr) mutable {
        HandleAssetLoadingResult(asset_ptr, std::move(asset_loader),
                                 gltf_asset_map_);
      })
      .KeptBy(this);
}

void AssetPtrMap::LoadGltfAsset(imp::AssetDefinition asset_definition,
                                std::unique_ptr<BaseAssetLoader> asset_loader) {
  auto load_options = GltfAsset::LoadOptions(
      {.collider_mode =
           imp::GltfState::ColliderMode::GLTF_COLLIDER_TRIANGLES_PER_MESH});
  view_.GetAssetManager()
      .LoadGltfAsset(asset_definition, load_options)
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<AssetPtr<GltfAsset>> asset_ptr) mutable {
        HandleAssetLoadingResult(asset_ptr, std::move(asset_loader),
                                 gltf_asset_map_);
      })
      .KeptBy(this);
}

absl::Status AssetPtrMap::ReleaseGltfAsset(std::intptr_t gltf_token) {
  auto asset_ptr = gltf_asset_map_.find(gltf_token);
  if (asset_ptr == gltf_asset_map_.end()) {
    return absl::NotFoundError(
        absl::StrFormat("Gltf asset with token %d is not cached.", gltf_token));
  }
  --asset_ptr->second.ref_count;
  if (asset_ptr->second.ref_count <= 0) {
    gltf_asset_map_.erase(asset_ptr);
  }
  return absl::OkStatus();
}

absl::StatusOr<AssetPtr<GltfAsset>> AssetPtrMap::GetStoredGltfAsset(
    std::intptr_t gltf_token) {
  auto asset_ptr = gltf_asset_map_.find(gltf_token);
  if (asset_ptr == gltf_asset_map_.end()) {
    return absl::NotFoundError("Gltf asset is not cached.");
  }
  return asset_ptr->second.asset;
}

absl::StatusOr<AssetPtr<ImageBasedLightingAsset>>
AssetPtrMap::GetStoredIblAsset(std::intptr_t ibl_token) {
  auto asset_ptr = ibl_asset_map_.find(ibl_token);
  if (asset_ptr == ibl_asset_map_.end()) {
    return absl::NotFoundError("Image based lighting asset is not cached.");
  }
  return asset_ptr->second.asset;
}

void AssetPtrMap::DestroyGltfAssets() { gltf_asset_map_.clear(); }

absl::Status AssetPtrMap::DisposeIblAssets() {
  view_.GetLightManager().ClearEnvironmentLight();
  ibl_asset_map_.clear();
  return absl::OkStatus();
}

// TODO: (broken link) - Accept URIs for asset paths, and remove / update this
// function.
std::string AssetPtrMap::GetAssetString(absl::string_view name) {
  // Check if the input string matches the URL pattern
  if (RE2::FullMatch(name, RE2(kRegexPattern))) {
    return std::string(name);
  } else {
    return std::string("file:///android_asset/").append(name);
  }
}

template <typename AssetT>
void AssetPtrMap::HandleAssetLoadingResult(
    absl::StatusOr<AssetPtr<AssetT>> asset_ptr,
    std::unique_ptr<BaseAssetLoader> asset_loader,
    absl::flat_hash_map<std::intptr_t, RefCountedAsset<AssetT>>& asset_map) {
  if (!asset_ptr.ok()) {
    asset_loader->OnFailure(asset_ptr.status().ToString());
    return;
  }

  std::intptr_t token = reinterpret_cast<std::intptr_t>(asset_ptr->Get());
  auto it = asset_map.find(token);
  if (it != asset_map.end()) {
    // Cached asset already exists, increment reference count.
    it->second.ref_count++;
  } else {
    asset_map[token] = {*asset_ptr, 1};
  }
  asset_loader->OnSuccess(token);
}

}  // namespace imp
