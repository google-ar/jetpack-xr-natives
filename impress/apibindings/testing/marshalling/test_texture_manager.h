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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_TEXTURE_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_TEXTURE_MANAGER_H_

#include <cstdint>
#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/texture_manager.h"
#include "core/render/texture.h"

namespace imp {

// Inherits from the real TextureManager for testing purposes.
class TestTextureManager : public TextureManager {
 public:
  explicit TestTextureManager(ImpressApiView& view);
  ~TestTextureManager() override = default;

  void LoadTexture(absl::string_view path,
                   std::unique_ptr<BaseAssetLoader> asset_loader) override;
  absl::StatusOr<std::intptr_t> BorrowReflectionTexture() override;
  absl::StatusOr<std::intptr_t> GetReflectionTextureFromIbl(
      std::intptr_t ibl_token) override;
  absl::StatusOr<BorrowedTexturePtr> BorrowTexture(
      std::intptr_t texture_handle) override;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_TEXTURE_MANAGER_H_
