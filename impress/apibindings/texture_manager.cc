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

#include "apibindings/texture_manager.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/asset_ptr_map.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/bindings_texture.h"
#include "apibindings/impress_api_view.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/lighting/environment_light.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/render/image_asset.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/render/texture_options.h"
#include "core/view/framework/lighting/light_manager.h"

namespace imp {

namespace {

constexpr TextureSamplerOptions kDefaultTextureSamplerOptions = {
    .wrap_mode = ::filament::TextureSampler::WrapMode::CLAMP_TO_EDGE,
    .mag_filter = ::filament::TextureSampler::MagFilter::LINEAR,
    .min_filter = ::filament::TextureSampler::MinFilter::LINEAR,
    .anisotropy = 1.0,
};

Future<OwnedTexturePtr> LoadTextureFromPath(TextureFactory& texture_factory,
                                            AssetManager& asset_manager,
                                            AssetPtrMap& asset_ptr_map,
                                            absl::string_view path) {
  return asset_manager.LoadImage(asset_ptr_map.GetAssetString(path))
      .Then([&texture_factory = texture_factory](
                AssetPtr<ImageAsset> image) -> absl::StatusOr<OwnedTexturePtr> {
        // Uploads the texture to the system.
        OwnedTexturePtr texture = texture_factory.CreateTexture(
            *image, imp::TextureGenerationOptions{},
            kDefaultTextureSamplerOptions);
        return texture;
      });
}

class TextureManagerImpl : public TextureManager {
 public:
  explicit TextureManagerImpl(ImpressApiView& view);
  ~TextureManagerImpl() override = default;

  void LoadTexture(absl::string_view path,
                   std::unique_ptr<BaseAssetLoader> asset_loader) override;
  absl::StatusOr<std::intptr_t> BorrowReflectionTexture() override;
  absl::StatusOr<std::intptr_t> GetReflectionTextureFromIbl(
      std::intptr_t ibl_token) override;
  absl::StatusOr<BorrowedTexturePtr> BorrowTexture(
      std::intptr_t texture_handle) override;

 private:
  ImpressApiView& view_;
};

}  // namespace

TextureManagerImpl::TextureManagerImpl(ImpressApiView& view) : view_(view) {}

void TextureManagerImpl::LoadTexture(
    absl::string_view path, std::unique_ptr<BaseAssetLoader> asset_loader) {
  LoadTextureFromPath(view_.GetTextureFactory(), view_.GetAssetManager(),
                      view_.GetAssetPtrMap(), path)
      .Then([this, asset_loader = std::move(asset_loader)](
                absl::StatusOr<OwnedTexturePtr> texture) mutable {
        if (texture.ok() && *texture != nullptr) {
          auto bindings_texture = new BindingsTexture(
              texture->Borrow(SmallSourceLocation::Current()));
          std::intptr_t texture_token = view_.ToJava(bindings_texture);
          view_.GetBindingsTextureMap().emplace(texture_token,
                                                std::move(*texture));
          asset_loader->OnSuccess(texture_token);
        } else {
          asset_loader->OnFailure(absl::StrFormat("Failed to load texture: %s.",
                                                  texture.status().message()));
        }
      })
      .KeptBy(&view_);
}

absl::StatusOr<std::intptr_t> TextureManagerImpl::BorrowReflectionTexture() {
  const EnvironmentLight* environment_light =
      view_.GetLightManager().GetEnvironmentLight();
  if (!environment_light) return absl::NotFoundError("No environment light.");

  std::optional<AssetPtr<ImageBasedLightingAsset>> ibl_asset =
      environment_light->GetReflectionIblAsset();
  if (!ibl_asset.has_value())
    return absl::NotFoundError("No reflection texture.");

  BorrowedTexturePtr reflections_texture =
      (*ibl_asset)->BorrowReflectionTexture();
  std::intptr_t reflections_texture_token =
      view_.ToJava(new BindingsTexture(std::move(reflections_texture)));
  return reflections_texture_token;
}

absl::StatusOr<std::intptr_t> TextureManagerImpl::GetReflectionTextureFromIbl(
    std::intptr_t ibl_token) {
  absl::StatusOr<AssetPtr<ImageBasedLightingAsset>> ibl_asset_ptr =
      view_.GetAssetPtrMap().GetStoredIblAsset(ibl_token);

  if (!ibl_asset_ptr.ok()) {
    return absl::NotFoundError(absl::StrFormat(
        "IBL asset is not cached: %s.", ibl_asset_ptr.status().message()));
  }

  BorrowedTexturePtr reflections_texture =
      ibl_asset_ptr.value()->BorrowSkyboxCubemap();
  std::intptr_t reflections_texture_token =
      view_.ToJava(new BindingsTexture(std::move(reflections_texture)));
  return reflections_texture_token;
}

absl::StatusOr<BorrowedTexturePtr> TextureManagerImpl::BorrowTexture(
    std::intptr_t texture_handle) {
  BindingsTexture* bindings_texture =
      view_.FromJava<BindingsTexture>(texture_handle);
  if (!bindings_texture) {
    return absl::InvalidArgumentError("Provided texture handle is not valid.");
  }

  BorrowedTexturePtr borrowed_texture =
      bindings_texture->GetTexture(SmallSourceLocation::Current());
  if (!borrowed_texture) {
    return absl::InvalidArgumentError(
        "Texture associated with handle is not valid.");
  }

  return borrowed_texture;
}

std::unique_ptr<TextureManager> CreateTextureManager(ImpressApiView& view) {
  return std::make_unique<TextureManagerImpl>(view);
}

}  // namespace imp
