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

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/editor/visualizers/material_visualizer.h"
#include "core/render/texture.h"
#include "core/render/texture_asset.h"
#include "core/view/base_view.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/string_map.h"

namespace imp::editor {

// Provides a thumbnail image to represent an asset for display in editor UI.
//
// If there is no thumbnail for the asset type available, it will fallback to an
// icon based on the assets extension.
class AssetThumbnailProvider {
 public:
  // Creates an AssetThumbnailProvider. Loads icons for various asset extensions
  // as part of creation.
  static Future<std::unique_ptr<AssetThumbnailProvider>> Create(BaseView& view);

  // Gets the thumbnail for the asset passed in as an Impress texture.
  //
  // Currently, only material definitions generate full thumbnails, everything
  // else uses an icon based on the extension.
  filament::Texture* GetThumbnailForResource(absl::string_view resource);

  // Sets the thumbnail for the resource.
  //
  // Material thumbnail overrides are not supported.
  void SetThumbnailOverride(absl::string_view resource, TexturePtr texture);

 private:
  static Future<absl::Status> LoadBasicIcon(BaseView& view,
                                            StringMap<TexturePtr>* icons,
                                            const imp::AssetDefinition& asset);

  AssetThumbnailProvider(BaseView& view, StringMap<TexturePtr> icons);

  BaseView& view_;
  // Default icons keyed by file extension.
  StringMap<TexturePtr> icons_;

  MaterialVisualizer material_visualizer_;
  // Futures for preparing material textures.
  StringMap<Future<Texture*>> material_thumbnail_futures_;
  // Resources that have had their icons overridden.
  StringMap<TexturePtr> resources_with_icons_;
  StringMap<Future<AssetPtr<TextureAsset>>> image_futures_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_THUMBNAIL_PROVIDER_H_
