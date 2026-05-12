/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_ASSET_PTR_MAP_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_ASSET_PTR_MAP_H_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_loader.h"
#include "core/assets/asset_ptr.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/utils/asset.h"
#include "imp.h"

namespace imp {

// Manages live asset objects used by the Impress Java API.
class AssetPtrMap : public Rememberer {
 public:
  explicit AssetPtrMap(BaseView& view);

  // Loads an image based lighting asset from the local assets folder or
  // a remote URL, and resolves the AssetLoader when it is ready.
  void LoadImageBasedLightingAsset(
      absl::string_view path, std::unique_ptr<BaseAssetLoader> asset_loader);

  // Loads an image based lighting asset pointer from a byte array, and resolves
  // the AssetLoader when it is ready.
  void LoadImageBasedLightingAsset(
      absl::Cord data, absl::string_view key,
      std::unique_ptr<BaseAssetLoader> asset_loader);

  // Loads an image based lighting asset pointer from an AssetDefinition, and
  // resolves the AssetLoader when it is ready.
  void LoadImageBasedLightingAsset(
      imp::AssetDefinition asset_definition,
      std::unique_ptr<BaseAssetLoader> asset_loader);

  // Releases a previously loaded image based lighting asset from the pointer
  // map if the reference count is 0, otherwise decrements the reference count.
  absl::Status ReleaseImageBasedLightingAsset(std::intptr_t ibl_token);

  // Loads the asset pointer of a glTF model from the local assets folder or
  // a remote URL, and resolves the AssetLoader when it is ready.
  void LoadGltfAsset(absl::string_view path,
                     std::unique_ptr<BaseAssetLoader> asset_loader);

  // Loads the asset pointer of a glTF model from a absl::Cord, and returns a
  // unique identifier for it when it is ready. The data will be managed by the
  // Impress resource system and will be destroyed when the AssetPtr associated
  // with the data is destroyed. Resolves the AssetLoader with the unique
  // identifier when it is ready.
  void LoadGltfAsset(absl::Cord data, absl::string_view key,
                     std::unique_ptr<BaseAssetLoader> asset_loader);

  // Loads the asset pointer of a glTF model from an AssetDefinition, and
  // resolves the AssetLoader when it is ready.
  void LoadGltfAsset(imp::AssetDefinition asset_definition,
                     std::unique_ptr<BaseAssetLoader> asset_loader);

  // Release a previously loaded glTF asset from the pointer map if the
  // reference count is 0, otherwise decrements the reference count.
  absl::Status ReleaseGltfAsset(std::intptr_t gltf_token);

  // Returns a previously loaded glTF asset from the pointer map if it exists,
  // otherwise returns an error status.
  absl::StatusOr<AssetPtr<GltfAsset>> GetStoredGltfAsset(
      std::intptr_t gltf_token);

  // Returns a previously loaded image based lighting asset from the pointer map
  // if it exists, otherwise returns an error status.
  absl::StatusOr<AssetPtr<ImageBasedLightingAsset>> GetStoredIblAsset(
      std::intptr_t ibl_token);

  // Disposes the glTF model assets associated with the Impress view.
  void DestroyGltfAssets();

  // Disposes the image based lighting assets associated with the Impress
  // view.
  absl::Status DisposeIblAssets();

  // Given a string, returns the corresponding path which will be used to load
  // the asset, and also as a key for later retrieval.
  static std::string GetAssetString(absl::string_view name);

 private:
  // Stores an asset and its reference count.
  template <typename AssetT>
  struct RefCountedAsset {
    AssetPtr<AssetT> asset;
    int32_t ref_count;
  };

  // Helper template to handle the result of loading an asset.
  template <typename AssetT>
  void HandleAssetLoadingResult(
      absl::StatusOr<AssetPtr<AssetT>> asset_ptr,
      std::unique_ptr<BaseAssetLoader> asset_loader,
      absl::flat_hash_map<std::intptr_t, RefCountedAsset<AssetT>>& asset_map);

  BaseView& view_;
  absl::flat_hash_map<std::intptr_t, RefCountedAsset<GltfAsset>>
      gltf_asset_map_;
  absl::flat_hash_map<std::intptr_t, RefCountedAsset<ImageBasedLightingAsset>>
      ibl_asset_map_;

  // Regular expression pattern for matching URLs.
  static constexpr absl::string_view kRegexPattern =
      "https?:\\/\\/"
      "(www\\.)?[-a-zA-Z0-9@:%._\\+~#=]{2,256}\\.[a-z]{2,4}\\b([-a-zA-Z0-9@:%_"
      "\\+.~#?&//=]*)";
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_ASSET_PTR_MAP_H_
