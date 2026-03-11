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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TEXTURE_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TEXTURE_MANAGER_H_

#include <cstdint>
#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_loader.h"
#include "core/render/texture.h"
#include "core/view/utils/asset.h"

namespace imp {

// Manages texture assets, loading, and lifetimes for the Jetpack XR Scene.
class TextureManager {
 public:
  virtual ~TextureManager() = default;

  // Loads a texture from the assets folder or a remote texture from a URL.
  virtual void LoadTexture(absl::string_view path,
                           std::unique_ptr<BaseAssetLoader> asset_loader) = 0;

  // Loads a texture from an asset definition.
  virtual void LoadTexture(imp::AssetDefinition asset_definition,
                           std::unique_ptr<BaseAssetLoader> asset_loader) = 0;

  // Borrows the reflection texture from the currently set environment IBL.
  virtual absl::StatusOr<std::intptr_t> BorrowReflectionTexture() = 0;

  // Borrows the reflection texture from the given environment IBL.
  virtual absl::StatusOr<std::intptr_t> GetReflectionTextureFromIbl(
      std::intptr_t ibl_token) = 0;

  // Borrows a texture by its handle, returning an error if not found.
  virtual absl::StatusOr<BorrowedTexturePtr> BorrowTexture(
      std::intptr_t texture_handle) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TEXTURE_MANAGER_H_
