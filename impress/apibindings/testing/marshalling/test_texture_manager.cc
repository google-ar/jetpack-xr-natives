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

#include "apibindings/testing/marshalling/test_texture_manager.h"

#include <cstdint>
#include <memory>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"
#include "core/render/texture.h"

namespace imp {

TestTextureManager::TestTextureManager(ImpressApiView& view) {}

void TestTextureManager::LoadTexture(
    absl::string_view path, std::unique_ptr<BaseAssetLoader> asset_loader) {
  IMP_LOG(imp::FATAL) << "TestTextureManager::LoadTexture unimplemented";
}

absl::StatusOr<std::intptr_t> TestTextureManager::BorrowReflectionTexture() {
  return absl::UnimplementedError(
      "TestTextureManager::BorrowReflectionTexture unimplemented");
}

absl::StatusOr<std::intptr_t> TestTextureManager::GetReflectionTextureFromIbl(
    std::intptr_t ibl_token) {
  return absl::UnimplementedError(
      "TestTextureManager::GetReflectionTextureFromIbl unimplemented");
}

absl::StatusOr<BorrowedTexturePtr> TestTextureManager::BorrowTexture(
    std::intptr_t texture_handle) {
  return absl::UnimplementedError(
      "TestTextureManager::BorrowTexture unimplemented");
}

}  // namespace imp
