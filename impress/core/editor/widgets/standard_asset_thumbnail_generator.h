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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_STANDARD_ASSET_THUMBNAIL_GENERATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_STANDARD_ASSET_THUMBNAIL_GENERATOR_H_

#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/editor/visualizers/material_visualizer.h"
#include "core/editor/widgets/asset_thumbnail_generator.h"
#include "core/render/texture.h"
#include "core/render/texture_asset.h"
#include "core/view/base_view.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/string_map.h"

namespace imp::editor {

// Standard implementation of AssetThumbnailGenerator.
class StandardAssetThumbnailGenerator : public AssetThumbnailGenerator {
 public:
  explicit StandardAssetThumbnailGenerator(BaseView& view);
  ~StandardAssetThumbnailGenerator() override = default;

  bool SupportsGeneration(absl::string_view resource) const override;
  void GenerateThumbnail(absl::string_view resource) override;
  absl::StatusOr<filament::Texture*> GetThumbnail(
      absl::string_view resource) override;

  // Helper to construct the generator and load icons for use in
  // AssetThumbnailProvider. Returns the fully prepared generator.
  static Future<std::unique_ptr<StandardAssetThumbnailGenerator>> Create(
      BaseView& view);

  // Transfers ownership of the default icons that were loaded during Create().
  StringMap<OwnedTexturePtr> ConsumeDefaultIcons();

 private:
  static Future<absl::Status> LoadBasicIcon(BaseView& view,
                                            StringMap<OwnedTexturePtr>* icons,
                                            const imp::AssetDefinition& asset,
                                            absl::string_view extension);

  BaseView& view_;
  MaterialVisualizer material_visualizer_;

  // Futures for preparing material textures.
  StringMap<Future<Texture*>> material_thumbnail_futures_;
  StringMap<Future<AssetPtr<TextureAsset>>> image_futures_;

  // Pre-loaded default icons, meant to be consumed by AssetThumbnailProvider.
  StringMap<OwnedTexturePtr> default_icons_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_STANDARD_ASSET_THUMBNAIL_GENERATOR_H_
