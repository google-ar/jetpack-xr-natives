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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_SKYBOX_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_SKYBOX_MANAGER_H_

#include <cstdint>
#include <memory>

#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/skybox_manager.h"

namespace imp {

// Inherits from the real SkyboxManager for testing purposes.
class TestSkyboxManager : public SkyboxManager {
 public:
  explicit TestSkyboxManager(ImpressApiView& view);
  ~TestSkyboxManager() override = default;

  void LoadImageBasedLightingAsset(
      absl::string_view path,
      std::unique_ptr<BaseAssetLoader> asset_loader) override;
  void LoadImageBasedLightingAsset(
      absl::Cord data, absl::string_view key,
      std::unique_ptr<BaseAssetLoader> asset_loader) override;
  absl::Status ReleaseImageBasedLightingAsset(std::intptr_t ibl_token) override;
  absl::Status SetEnvironmentLight(std::intptr_t ibl_token) override;
  void ClearEnvironmentLight() override;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_SKYBOX_MANAGER_H_
