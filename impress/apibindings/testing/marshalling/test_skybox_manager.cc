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

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"

namespace imp {

TestSkyboxManager::TestSkyboxManager(ImpressApiView& view) {}

void TestSkyboxManager::LoadImageBasedLightingAsset(
    absl::string_view path, std::unique_ptr<BaseAssetLoader> asset_loader) {
  IMP_LOG(imp::FATAL)
      << "TestSkyboxManager::LoadImageBasedLightingAsset(path) unimplemented";
}

void TestSkyboxManager::LoadImageBasedLightingAsset(
    absl::Cord data, absl::string_view key,
    std::unique_ptr<BaseAssetLoader> asset_loader) {
  IMP_LOG(imp::FATAL)
      << "TestSkyboxManager::LoadImageBasedLightingAsset(data) unimplemented";
}

absl::Status TestSkyboxManager::ReleaseImageBasedLightingAsset(
    std::intptr_t ibl_token) {
  return absl::UnimplementedError(
      "TestSkyboxManager::ReleaseImageBasedLightingAsset unimplemented");
}

absl::Status TestSkyboxManager::SetEnvironmentLight(std::intptr_t ibl_token) {
  return absl::UnimplementedError(
      "TestSkyboxManager::SetEnvironmentLight unimplemented");
}

absl::Status TestSkyboxManager::ClearEnvironmentLight() {
  return absl::UnimplementedError(
      "TestSkyboxManager::ClearEnvironmentLight unimplemented");
}

}  // namespace imp
