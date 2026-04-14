/*
 * Copyright 2026 Google LLC
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

#include "core/editor/widgets/asset_thumbnail_provider.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/common/file_helpers.h"
#include "core/editor/widgets/asset_thumbnail_generator.h"
#include "core/editor/widgets/asset_type_helpers.h"
#include "core/render/texture.h"
#include "core/view/utils/string_map.h"

namespace imp::editor {

AssetThumbnailProvider::AssetThumbnailProvider(
    StringMap<OwnedTexturePtr> icons,
    std::unique_ptr<AssetThumbnailGenerator> generator)
    : icons_(std::move(icons)), generator_(std::move(generator)) {
  // extract pending icon if present
  for (auto it = icons_.begin(); it != icons_.end(); ++it) {
    if (it->first == kPendingIconName) {
      pending_icon_ = std::move(const_cast<OwnedTexturePtr&>(it->second));
      icons_.erase(it);
      break;
    }
  }
}

filament::Texture* AssetThumbnailProvider::GetThumbnailForResource(
    absl::string_view resource) {
  // If there's an icon overridden for this resource, return it.
  auto itr = resources_with_icons_.find(resource);
  if (itr != resources_with_icons_.end()) {
    return itr->second->GetTexture();
  }

  absl::string_view extension = GetExtensionFromFilename(resource);

  // Try generator for supported extensions.
  if ((extension == kPngExt || extension == kMaterialDefinitionExt) &&
      generator_) {
    absl::StatusOr<filament::Texture*> texture =
        generator_->GetThumbnail(resource);

    // If the texture is ready, return it.
    if (texture.ok() && (*texture != nullptr)) {
      return *texture;
    }

    // Tf texture generation is not triggered yet, or failed, try to generate
    // it.
    if (!texture.ok() && generator_->SupportsGeneration(resource)) {
      generator_->GenerateThumbnail(resource);
    }

    // Fallback to default icons
    auto icon_it = icons_.find(extension);
    if (icon_it != icons_.end()) {
      return icon_it->second->GetTexture();
    }

    return pending_icon_ ? pending_icon_->GetTexture() : nullptr;
  }

  // Fallback to default icons
  auto icon_it = icons_.find(extension);
  if (icon_it != icons_.end()) {
    return icon_it->second->GetTexture();
  }

  // File icon fallback (empty string key)
  auto file_it = icons_.find(kFileIconName);
  if (file_it != icons_.end()) {
    return file_it->second->GetTexture();
  }

  return nullptr;
}

void AssetThumbnailProvider::SetThumbnailOverride(absl::string_view resource,
                                                  OwnedTexturePtr texture) {
  resources_with_icons_.insert_or_assign(std::string(resource),
                                         std::move(texture));
}

void AssetThumbnailProvider::SetDefaultIcon(absl::string_view extension,
                                            OwnedTexturePtr icon) {
  icons_.insert_or_assign(std::string(extension), std::move(icon));
}

void AssetThumbnailProvider::SetPendingIcon(OwnedTexturePtr icon) {
  pending_icon_ = std::move(icon);
}

}  // namespace imp::editor
