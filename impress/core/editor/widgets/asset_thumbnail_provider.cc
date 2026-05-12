// Copyright 2024 Google LLC
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

#include "core/editor/widgets/asset_thumbnail_provider.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/file_helpers.h"
#include "core/editor/widgets/asset_type_helpers.h"
#include "core/editor/widgets/icons/texture_assets.h"
#include "core/render/image_asset.h"
#include "core/render/texture.h"
#include "core/render/texture_asset.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/proto_asset.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/string_map.h"

namespace imp::editor {

Future<std::unique_ptr<AssetThumbnailProvider>> AssetThumbnailProvider::Create(
    BaseView& view) {
  std::unique_ptr<StringMap<TexturePtr>> icons =
      std::make_unique<StringMap<TexturePtr>>();
  StringMap<TexturePtr>* icons_ptr = icons.get();

  Future<absl::Status> result(absl::OkStatus());

  result =
      result.Combine(LoadBasicIcon(view, icons_ptr, texture_data::kFilePng));
  result =
      result.Combine(LoadBasicIcon(view, icons_ptr, texture_data::kScenePng));
  result =
      result.Combine(LoadBasicIcon(view, icons_ptr, texture_data::kModelPng));
  result = result.Combine(
      LoadBasicIcon(view, icons_ptr, texture_data::kMaterialPng));
  result =
      result.Combine(LoadBasicIcon(view, icons_ptr, texture_data::kImagePng));
  result =
      result.Combine(LoadBasicIcon(view, icons_ptr, texture_data::kPendingPng));

  return result.Then([&view, icons = std::move(icons)]() mutable
                     -> std::unique_ptr<AssetThumbnailProvider> {
    return absl::WrapUnique(
        new AssetThumbnailProvider(view, std::move(*icons)));
  });
}

Future<absl::Status> AssetThumbnailProvider::LoadBasicIcon(
    BaseView& view, StringMap<TexturePtr>* icons,
    const imp::AssetDefinition& asset) {
  return view.GetAssetManager().LoadImage(asset).Then(
      [&view, icons](AssetPtr<ImageAsset> icon_image) mutable {
        TexturePtr icon = view.GetTextureFactory().CreateTexture(*icon_image);
        icons->insert_or_assign(std::string(icon_image->GetName()),
                                std::move(icon));
      });
}

AssetThumbnailProvider::AssetThumbnailProvider(BaseView& view,
                                               StringMap<TexturePtr> icons)
    : view_(view), icons_(std::move(icons)), material_visualizer_(view) {}

filament::Texture* AssetThumbnailProvider::GetThumbnailForResource(
    absl::string_view resource) {
  // If there's an icon overridden for this resource, return it.
  auto itr = resources_with_icons_.find(resource);
  if (itr != resources_with_icons_.end()) {
    return itr->second->GetTexture();
  }

  absl::string_view extension = GetExtensionFromFilename(resource);

  // TODO: Build a system for generating thumbnails for different
  // asset types. Ideally, this system should generate the thumbnail just once
  // per asset, and likely blit it onto a texture atlas using ShelfAtlasPacker
  // with an LRU cache.
  // TODO: Add scuba tests for all of the different thumbnail
  // types.
  if (extension == kPngExt) {
    auto itr = image_futures_.find(resource);

    // We've already either started loading this image or failed to load it.
    if (itr != image_futures_.end()) {
      Future<AssetPtr<TextureAsset>>& image_future = itr.value();

      // The image texture is in the process of loading.
      if (!image_future.Ready()) {
        return icons_.at(texture_data::kImagePng.GetUrl())->GetTexture();
      }

      // The image texture failed to load, remove the future from the map so
      // we can try again.
      if (!image_future.Get().ok()) {
        IMP_LOG(imp::ERROR) << "Image loading failed: "
                   << image_future.Get().status().ToString();
        image_futures_.erase(itr);
        return icons_.at(texture_data::kImagePng.GetUrl())->GetTexture();
      }

      // The image texture has finished loading, return the texture.
      AssetPtr<TextureAsset> texture_asset = image_future.Get().value();
      

      return texture_asset->GetFilamentTexture();
    }

    Future<AssetPtr<TextureAsset>> image_future =
        view_.GetAssetManager().LoadTexture(resource);

    image_futures_.insert_or_assign(std::string(resource), image_future);
    return icons_.at(texture_data::kPendingPng.GetUrl())->GetTexture();
  } else if (extension == kGltfExt) {
    return icons_.at(texture_data::kModelPng.GetUrl())->GetTexture();
  } else if (extension == kGlbExt) {
    return icons_.at(texture_data::kModelPng.GetUrl())->GetTexture();
  } else if (extension == kCmatExt) {
    return icons_.at(texture_data::kMaterialPng.GetUrl())->GetTexture();
  } else if (extension == kIsfExt) {
    return icons_.at(texture_data::kScenePng.GetUrl())->GetTexture();
  } else if (extension == kMaterialDefinitionExt) {
    auto itr = material_thumbnail_futures_.find(resource);

    // We've already either started loading this material or failed to load
    // it.
    if (itr != material_thumbnail_futures_.end()) {
      Future<Texture*>& material_future = itr.value();

      // The material texture is in the process of loading.
      if (!material_future.Ready()) {
        return icons_.at(texture_data::kPendingPng.GetUrl())->GetTexture();
      }

      // The material texture failed to load, remove the future from the map so
      // we can try again.
      if (!material_future.Get().ok()) {
        IMP_LOG(imp::ERROR) << "Material Visualizer failed: "
                   << material_future.Get().status().ToString();
        material_thumbnail_futures_.erase(itr);
        return icons_.at(texture_data::kPendingPng.GetUrl())->GetTexture();
      }

      // The material texture has finished loading, return the texture.
      Texture* result = material_future.Get().value();
      

      return result->GetTexture();
    }

    Future<Texture*> material_future =
        view_.GetAssetManager().LoadProto<MaterialDefinition>(resource).Then(
            [this](AssetPtr<ProtoAsset<MaterialDefinition>>
                       material_definition) mutable {
              return material_visualizer_.GetMaterialPreview(
                  material_definition);
            });

    material_thumbnail_futures_.insert_or_assign(std::string(resource),
                                                 material_future);
    return icons_.at(texture_data::kPendingPng.GetUrl())->GetTexture();
  } else {
    return icons_.at(texture_data::kFilePng.GetUrl())->GetTexture();
  }
}

void AssetThumbnailProvider::SetThumbnailOverride(absl::string_view resource,
                                                  TexturePtr texture) {
  resources_with_icons_.insert_or_assign(std::string(resource),
                                         std::move(texture));
}

}  // namespace imp::editor
