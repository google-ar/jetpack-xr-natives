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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_IBL_ASSET_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_IBL_ASSET_H_

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/base/nullability.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/invocable.h"
#include "core/common/small_source_location.h"
#include "core/lighting/image_based_lighting_types.h"
#include "core/render/texture.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"

namespace imp {

// ImageBasedLightingAsset contains data that can be used
// for indirect lighting and skybox rendering. ImageBasedLightingAsset can be
// built through `imp_imaged_based_lighting` rule. See example in:
// third_party/impress/build_tools/imp_image_based_lighting.bzl
// Similar to other assets(for example, MaterialAsset), ImageBasedLightingAsset
// can be loaded through AssetManager.
//
// TODO: Add a sample project for demonstrating the new indirect
// lighting pipeline.
// TODO: Add tests for ImageBasedLightingAsset.
class ImageBasedLightingAsset {
 public:
  // Specifies a Load function that overrides the default Load function of
  // ImageBasedLightingAsset.
  // Example usage:
  // struct MyCustomLoader : public CustomLoader;
  // auto my_custom_loader = MyCustomLoader();
  // AssetPtr<ImageBasedLightingAsset> asset =
  //  AssetManager.LoadAsset<ImageBasedLightingAsset>(
  //    asset_definition, my_custom_loader);
  struct CustomLoader {
    virtual ~CustomLoader() = default;
    virtual Future<std::unique_ptr<ImageBasedLightingAsset>> Load(
        BaseView* view, absl::string_view asset_url,
        Future<resources::Resource> resource_future) = 0;
  };

  ~ImageBasedLightingAsset();

  ImageBasedLightingAsset(
      std::unique_ptr<SphericalHarmonics> spherical_harmonics,
      OwnedTexturePtr ibl_cubemap, OwnedTexturePtr skybox_cubemap,
      std::optional<std::string_view> asset_url);

  ImageBasedLightingAsset(
      BaseView& view, std::unique_ptr<SphericalHarmonics> spherical_harmonics,
      ImageBasedLightingAssetCubemapImages cubemap_images,
      std::optional<std::string_view> asset_url);

  static Future<std::unique_ptr<ImageBasedLightingAsset>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future);

  static Future<std::unique_ptr<ImageBasedLightingAsset>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future, CustomLoader& custom_loader);

  // Get the cubemap texture for lighting. This could return an invalid pointer
  // if IBL cubemap was not built or failed to load.
  ABSL_DEPRECATED("Use BorrowLightingCubemap instead.")
  Texture* GetLightingCubemap() const;

  // Get the cubemap texture for Skybox. This could return an invalid pointer if
  // skybox cubemap was not built or failed to load.
  ABSL_DEPRECATED("Use BorrowSkyboxCubemap instead.")
  Texture* GetSkyboxCubemap() const;

  // Get the cubemap texture for reflections. This returns the skybox cubemap
  // if it exists, otherwise it returns the lighting cubemap. This could return
  // an invalid pointer if both cubemaps were not built or failed to load.
  ABSL_DEPRECATED("Use BorrowReflectionTexture instead.")
  Texture* GetReflectionTexture() const;

  // Get the cubemap texture for lighting. This could return an invalid pointer
  // if IBL cubemap was not built or failed to load.
  BorrowedTexturePtr BorrowLightingCubemap(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Get the cubemap texture for Skybox. This could return an invalid pointer if
  // skybox cubemap was not built or failed to load.
  BorrowedTexturePtr BorrowSkyboxCubemap(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Get the cubemap texture for reflections. This returns the skybox cubemap
  // if it exists, otherwise it returns the lighting cubemap. This could return
  // an invalid pointer if both cubemaps were not built or failed to load.
  BorrowedTexturePtr BorrowReflectionTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Returns the asset url if the ImageBasedLightingAsset is loaded through
  // AssetManager from a file. Otherwise, std::nullopt will be returned.
  std::optional<absl::string_view> GetAssetUrl() const { return asset_url_; }

  // Get the SphericalHarmonics. This could return an invalid pointer if
  // spherical harmonics data was not built or failed to load.
  const SphericalHarmonics* GetSphericalHarmonics() const {
    return spherical_harmonics_.get();
  }

 private:
  // Loads spherical harmonics information from a buffer.
  // The buffer is expected to be text and has the exact same format of
  // cmgen's spherical harmonics output:
  // (broken link)
  static absl::StatusOr<std::unique_ptr<SphericalHarmonics>>
  LoadSphericalHarmonics(const BufferAccess& data);

  // Loads ibl cubemap texture from a list of CubemapLevelImageContents.
  static OwnedTexturePtr LoadIblCubemap(
      const Context& context, TextureFactory& texture_factory,
      std::vector<CubemapLevelImageContents> ibl_cubemap_level_image_contents,
      filament::Engine& engine);

  // Loads skybox cubemap texture from a cubemap.
  static OwnedTexturePtr LoadSkyboxCubemap(
      const Context& context, TextureFactory& texture_factory,
      filament::Engine& engine,
      CubemapLevelImageContents skybox_cubemap_level_image_contents);

  // Loads a list of CubemapLevelImageContents, representing IBL cubemaps for
  // each cubemap level, from a map of buffers. The keys should be the file
  // names of the cubemap levels that's generated by cmgen.
  static Future<std::vector<CubemapLevelImageContents>>
  LoadIblCubemapLevelImageContents(
      const Context& context, TextureFactory& texture_factory,
      filament::Engine& engine,
      const StringMap<resources::Resource>& filename_to_resource);

  // Loads a CubemapLevelImageContents, representing the skybox cubemap, from a
  // map of buffers. The keys should be the file names of the cubemap levels
  // that's generated by cmgen.
  static Future<CubemapLevelImageContents> LoadSkyboxCubemapLevelImageContents(
      const Context& context, TextureFactory& texture_factory,
      filament::Engine& engine,
      const StringMap<resources::Resource>& filename_to_resource);

  // Given spherical harmonics, IBL CubemapLevelImageContents, and an optional
  // skybox CubemapLevelImageContents, returns an ImageBasedLightingAsset.
  // Serializes the resultant ImageasedLightingAsset and adds it to the
  // SplitEngineSerializer, if it exists.
  static Future<std::unique_ptr<ImageBasedLightingAsset>>
  SerializeAndConstructImageBasedLightingAsset(
      BaseView& view,
      /*absl_nonnull*/  std::unique_ptr<SphericalHarmonics> spherical_harmonics,
      ImageBasedLightingAssetCubemapImages cubemap_images,
      std::optional<std::string> asset_url);

  std::unique_ptr<SphericalHarmonics> spherical_harmonics_;
  // Pre-filtered mip map cubemap, intended for IBL.
  OwnedTexturePtr ibl_cubemap_;
  // Unfiltered cubemap, intended for skyboxes.
  OwnedTexturePtr skybox_cubemap_;

  std::optional<std::string> asset_url_;
  Invocable<void()> on_destroy_callback_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_IBL_ASSET_H_
