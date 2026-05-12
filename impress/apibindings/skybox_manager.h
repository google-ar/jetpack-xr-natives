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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_SKYBOX_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_SKYBOX_MANAGER_H_

#include <cstdint>
#include <memory>

#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_loader.h"

namespace imp {

// Manages IBL/Skybox assets and environment light settings for the
// Jetpack XR Scene.
class SkyboxManager {
 public:
  virtual ~SkyboxManager() = default;

  // Loads the asset pointer of an IBL asset from the local assets folder or
  // a remote URL, and returns a unique identifier for it when it is ready.
  virtual void LoadImageBasedLightingAsset(
      absl::string_view path,
      std::unique_ptr<BaseAssetLoader> asset_loader) = 0;

  // Loads the asset pointer of an IBL asset from a byte array, and returns a
  // unique identifier for it when it is ready.
  virtual void LoadImageBasedLightingAsset(
      absl::Cord data, absl::string_view key,
      std::unique_ptr<BaseAssetLoader> asset_loader) = 0;

  // Releases the asset pointer of a previously loaded image based lighting
  // asset if the reference count is 0, otherwise decrements the reference
  // count.
  virtual absl::Status ReleaseImageBasedLightingAsset(
      std::intptr_t ibl_token) = 0;

  // Sets the preferred IBL asset to be used by the system.
  virtual absl::Status SetEnvironmentLight(std::intptr_t ibl_token) = 0;

  // Clears the preferred IBL asset to be used by the system.
  virtual void ClearEnvironmentLight() = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_SKYBOX_MANAGER_H_
