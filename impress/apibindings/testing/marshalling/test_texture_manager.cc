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
#include <string>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/testing/marshalling/texture_test_context.h"
#include "core/render/texture.h"
#include "core/view/utils/asset.h"

namespace imp {

TestTextureManager::TestTextureManager(ImpressApiView& view) {}

void TestTextureManager::LoadTexture(
    absl::string_view path, std::unique_ptr<BaseAssetLoader> asset_loader) {
  TextureTestContext& context = TextureTestContext::Get();
  context.load_texture_asset_path.actual_path = std::string(path);

  

  if (asset_loader != nullptr) {
    if (!context.load_texture_asset_path.failure_message.empty()) {
      asset_loader->OnFailure(context.load_texture_asset_path.failure_message);
    } else {
      asset_loader->OnSuccess(context.load_texture_asset_path.success_token);
    }
  }
}

void TestTextureManager::LoadTexture(
    imp::AssetDefinition asset_definition,
    std::unique_ptr<BaseAssetLoader> asset_loader) {
  IMP_LOG(imp::FATAL) << "TestTextureManager::LoadTexture(imp::AssetDefinition "
                "asset_definition, std::unique_ptr<BaseAssetLoader> "
                "asset_loader) unimplemented since it is only used for unit "
                "tests.";
}

absl::StatusOr<std::intptr_t> TestTextureManager::BorrowReflectionTexture() {
  TextureTestContext& context = TextureTestContext::Get();
  context.borrow_reflection_texture.actual_call = true;
  

  if (context.borrow_reflection_texture.success_token != 0L) {
    return context.borrow_reflection_texture.success_token;
  }
  return absl::NotFoundError("Mock Reflection Texture Not Found.");
}

absl::StatusOr<std::intptr_t> TestTextureManager::GetReflectionTextureFromIbl(
    std::intptr_t ibl_token) {
  TextureTestContext& context = TextureTestContext::Get();
  context.get_reflection_texture_from_ibl.actual_ibl_token = ibl_token;

  

  if (context.get_reflection_texture_from_ibl.success_token != 0L) {
    return context.get_reflection_texture_from_ibl.success_token;
  }
  return absl::NotFoundError("Mock Reflection Texture from IBL Not Found.");
}

absl::StatusOr<BorrowedTexturePtr> TestTextureManager::BorrowTexture(
    std::intptr_t texture_handle) {
  TextureTestContext& context = TextureTestContext::Get();
  context.borrow_texture.actual_handle = texture_handle;

  

  if (texture_handle == 0L) {
    return absl::InvalidArgumentError("Cannot borrow texture from handle 0.");
  }

  return BorrowedTexturePtr{};
}

}  // namespace imp
