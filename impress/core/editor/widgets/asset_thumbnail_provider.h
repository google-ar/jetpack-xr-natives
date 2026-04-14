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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_THUMBNAIL_PROVIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_THUMBNAIL_PROVIDER_H_

#include <memory>
#include <string>

#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/editor/widgets/asset_thumbnail_generator.h"
#include "core/render/texture.h"
#include "core/view/utils/string_map.h"

namespace imp::editor {

// Provides a thumbnail image to represent an asset for display in editor UI.
//
// If there is no thumbnail for the asset type available, it will fallback to an
// icon based on the assets extension.
class AssetThumbnailProvider {
 public:
  AssetThumbnailProvider(StringMap<OwnedTexturePtr> icons,
                         std::unique_ptr<AssetThumbnailGenerator> generator);

  // Gets the thumbnail for the asset passed in as an Impress texture.
  //
  // Checks overrides, then generator, then default icons.
  filament::Texture* GetThumbnailForResource(absl::string_view resource);

  // Sets the thumbnail for the resource.
  void SetThumbnailOverride(absl::string_view resource,
                            OwnedTexturePtr texture);

  // Sets the default icon for a specific extension.
  // Use empty string for fallback file icon.
  void SetDefaultIcon(absl::string_view extension, OwnedTexturePtr icon);

  // Sets the pending icon.
  void SetPendingIcon(OwnedTexturePtr icon);

 private:
  // Default icons keyed by file extension.
  StringMap<OwnedTexturePtr> icons_;

  // Pending icon separate from icons map for easy access.
  OwnedTexturePtr pending_icon_;

  // Generator for dynamic thumbnails.
  std::unique_ptr<AssetThumbnailGenerator> generator_;

  // Resources that have had their icons overridden.
  StringMap<OwnedTexturePtr> resources_with_icons_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_THUMBNAIL_PROVIDER_H_
