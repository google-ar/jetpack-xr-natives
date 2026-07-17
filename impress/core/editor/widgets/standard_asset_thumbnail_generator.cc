/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "core/editor/widgets/standard_asset_thumbnail_generator.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/proto_asset.h"
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
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/string_map.h"

namespace imp::editor {

// static
Future<absl::Status> StandardAssetThumbnailGenerator::LoadBasicIcon(
    BaseView& view, StringMap<OwnedTexturePtr>* icons,
    const imp::AssetDefinition& asset, absl::string_view extension) {
  return view.GetAssetManager().LoadImage(asset).Then(
      [&view, icons, extension = std::string(extension)](
          AssetPtr<ImageAsset> icon_image) mutable {
        OwnedTexturePtr icon =
            view.GetTextureFactory().CreateTexture(*icon_image);
        icons->insert_or_assign(extension, std::move(icon));
      });
}

// static
Future<std::unique_ptr<StandardAssetThumbnailGenerator>>
StandardAssetThumbnailGenerator::Create(BaseView& view) {
  auto icons = std::make_unique<StringMap<OwnedTexturePtr>>();
  StringMap<OwnedTexturePtr>* icons_ptr = icons.get();

  Future<absl::Status> result(absl::OkStatus());

  // Default file icon (empty string key)
  result = result.Combine(
      LoadBasicIcon(view, icons_ptr, texture_data::kFilePng, kFileIconName));
  result = result.Combine(
      LoadBasicIcon(view, icons_ptr, texture_data::kScenePng, kIsfExt));
  // Map both glTF and GLB to the model icon
  result = result.Combine(
      LoadBasicIcon(view, icons_ptr, texture_data::kModelPng, kGltfExt));
  result = result.Combine(
      LoadBasicIcon(view, icons_ptr, texture_data::kModelPng, kGlbExt));
  result = result.Combine(
      LoadBasicIcon(view, icons_ptr, texture_data::kMaterialPng, kCmatExt));
  result = result.Combine(
      LoadBasicIcon(view, icons_ptr, texture_data::kImagePng, kPngExt));
  // "pending" icon
  result = result.Combine(LoadBasicIcon(
      view, icons_ptr, texture_data::kPendingPng, kPendingIconName));

  return result.Then([icons = std::move(icons), view_ptr = &view]() mutable {
    auto generator =
        std::make_unique<StandardAssetThumbnailGenerator>(*view_ptr);
    generator->default_icons_ = std::move(*icons);
    return generator;
  });
}

StringMap<OwnedTexturePtr>
StandardAssetThumbnailGenerator::ConsumeDefaultIcons() {
  return std::move(default_icons_);
}

StandardAssetThumbnailGenerator::StandardAssetThumbnailGenerator(BaseView& view)
    : view_(view), material_visualizer_(view) {}

bool StandardAssetThumbnailGenerator::SupportsGeneration(
    absl::string_view resource) const {
  absl::string_view extension = GetExtensionFromFilename(resource);
  return extension == kPngExt || extension == kMaterialDefinitionExt;
}

void StandardAssetThumbnailGenerator::GenerateThumbnail(
    absl::string_view resource) {
  absl::string_view extension = GetExtensionFromFilename(resource);

  if (extension == kPngExt) {
    if (image_futures_.contains(resource)) {
      return;
    }
    Future<AssetPtr<TextureAsset>> image_future =
        view_.GetAssetManager().LoadTexture(resource);
    image_futures_.insert_or_assign(std::string(resource), image_future);
  } else if (extension == kMaterialDefinitionExt) {
    if (material_thumbnail_futures_.contains(resource)) {
      return;
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
  }
}

absl::StatusOr<filament::Texture*>
StandardAssetThumbnailGenerator::GetThumbnail(absl::string_view resource) {
  absl::string_view extension = GetExtensionFromFilename(resource);

  if (extension == kPngExt) {
    auto itr = image_futures_.find(resource);
    if (itr == image_futures_.end()) {
      // Not requested yet.
      return absl::NotFoundError("Image not requested yet");
    }

    Future<AssetPtr<TextureAsset>>& image_future = itr.value();
    if (!image_future.Ready()) {
      return nullptr;
    }

    if (!image_future.Get().ok()) {
      IMP_LOG(imp::ERROR) << "Image loading failed: "
                 << image_future.Get().status().ToString();
      image_futures_.erase(itr);
      return absl::InternalError("Image loading failed");
    }

    AssetPtr<TextureAsset> texture_asset = image_future.Get().value();
    
    return texture_asset->GetFilamentTexture();

  } else if (extension == kMaterialDefinitionExt) {
    auto itr = material_thumbnail_futures_.find(resource);
    if (itr == material_thumbnail_futures_.end()) {
      return absl::NotFoundError("Material not requested yet");
    }

    Future<Texture*>& material_future = itr.value();
    if (!material_future.Ready()) {
      return nullptr;
    }

    if (!material_future.Get().ok()) {
      IMP_LOG(imp::ERROR) << "Material Visualizer failed details: "
                 << material_future.Get().status().ToString();
      material_thumbnail_futures_.erase(itr);
      return absl::InternalError("Material Visualizer failed");
    }

    Texture* result = material_future.Get().value();
    
    return result->GetTexture();
  }

  // Not handled by this generator.
  return absl::InvalidArgumentError("Resource not handled by this generator");
}

}  // namespace imp::editor
