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

#include "apibindings/testing/marshalling/test_skybox_manager.h"

#include <cstdint>
#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/testing/marshalling/jni_test_utils.h"
#include "apibindings/testing/marshalling/skybox_test_context.h"

namespace imp {

TestSkyboxManager::TestSkyboxManager(ImpressApiView& view) {}

void TestSkyboxManager::LoadImageBasedLightingAsset(
    absl::string_view path, std::unique_ptr<BaseAssetLoader> asset_loader) {
  SkyboxTestContext& context = SkyboxTestContext::Get();
  context.load_image_based_lighting_asset_path.actual_path = std::string(path);

  

  if (asset_loader != nullptr) {
    if (!context.load_image_based_lighting_asset_path.failure_message.empty()) {
      asset_loader->OnFailure(
          context.load_image_based_lighting_asset_path.failure_message);
    } else {
      asset_loader->OnSuccess(
          context.load_image_based_lighting_asset_path.success_token);
    }
  }
}

void TestSkyboxManager::LoadImageBasedLightingAsset(
    absl::Cord data, absl::string_view key,
    std::unique_ptr<BaseAssetLoader> asset_loader) {
  SkyboxTestContext& context = SkyboxTestContext::Get();
  context.load_image_based_lighting_asset_bytes.actual_key = std::string(key);
  

  if (context.load_image_based_lighting_asset_bytes.expect_test_pattern) {
    EXPECT_OK(VerifyTestPattern(
        data, context.load_image_based_lighting_asset_bytes.expected_size))
        << "IBL Asset JNI Marshalling Data Corruption Detected";
  }

  if (asset_loader != nullptr) {
    if (!context.load_image_based_lighting_asset_bytes.failure_message
             .empty()) {
      asset_loader->OnFailure(
          context.load_image_based_lighting_asset_bytes.failure_message);
    } else {
      asset_loader->OnSuccess(
          context.load_image_based_lighting_asset_bytes.success_token);
    }
  }
}

absl::Status TestSkyboxManager::ReleaseImageBasedLightingAsset(
    std::intptr_t ibl_token) {
  SkyboxTestContext& context = SkyboxTestContext::Get();
  context.release_image_based_lighting_asset.actual_token = ibl_token;
  
  return absl::OkStatus();
}

absl::Status TestSkyboxManager::SetEnvironmentLight(std::intptr_t ibl_token) {
  SkyboxTestContext& context = SkyboxTestContext::Get();
  context.set_environment_light.actual_token = ibl_token;
  
  return absl::OkStatus();
}

void TestSkyboxManager::ClearEnvironmentLight() {
  SkyboxTestContext& context = SkyboxTestContext::Get();
  context.clear_environment_light.actual_clear = true;
  
}

}  // namespace imp
