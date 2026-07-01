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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_FACTORY_H_

#include <cstdint>
#include <optional>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/assets/asset_ptr.h"
#include "core/common/small_source_location.h"
#include "core/image/image_contents.h"
#include "core/image/owned_image_content.h"
#include "core/math/vec.h"
#include "core/render/content_security_level.h"
#include "core/render/image_asset.h"
#include "core/render/texture.h"
#include "core/render/texture_asset.h"
#include "core/render/texture_options.h"
#include "core/view/base_view.h"

namespace imp {

class View;

// A factory for creating various types of Texture assets, accessed through
// TexturePtrs.
class TextureFactory {
 public:
  using MinFilter = filament::TextureSampler::MinFilter;
  using MagFilter = filament::TextureSampler::MagFilter;
  using WrapMode = filament::TextureSampler::WrapMode;
  using Format = filament::Texture::InternalFormat;
  using Usage = filament::Texture::Usage;
  using SamplerType = filament::Texture::Sampler;
  using ExternalImageHandle = filament::Texture::ExternalImageHandle;

  // Used for configuring how textures are created.
  // These setting are mirroring the available settings in
  // filament::Texture::Builder:
  // third_party/filament/filament/include/filament/Texture.h
  //
  // TODO: Refactor CreateTexture() to use this struct.
  struct TextureCreationSettings {
    // Width of the texture. Defaults to be 256.
    uint32_t width = 256;
    // Height of the texture. Defaults to be 256.
    uint32_t height = 256;
    // Format of the texture. Defaults to be Format::RGBA8.
    Format format = Format::RGBA8;
    // Depth of the texture.
    std::optional<uint32_t> depth;
    // Number of mip map levels of the texture.
    std::optional<uint8_t> levels;
    // Usage of the texture.
    std::optional<Usage> usage;
    // With Metal, the id<MTLTexture> object should be cast to an intptr_t
    // using CFBridgingRetain to transfer ownership to Filament. Filament
    // will release ownership of the texture object when the Filament texture
    // is destroyed. The width, height, levels, and format should match what the
    // metal texture returns.
    std::optional<intptr_t> native_texture_id;
    // Sampler options for the texture.
    std::optional<TextureSamplerOptions> sampler_options;
    // Optional name for the texture.
    std::optional<absl::string_view> name;
  };

  TextureFactory(BaseView& view);

  // TODO: (broken link) - Refactor CreateTexture functions to reduce duplication
  // and volume of overloads. Make it clear somehow which calls can be used
  // with Split Engine.

  // Create an external texture with a native stream, whose type depends on the
  // platform. Returns a null handle if it fails to create the texture.
  // Dimension information can be optionally provided purely for informational
  // purposes.
  //   Platform | native_stream type
  //   :--------|:----------------------------------------------:
  //   Android  | `android/graphics/SurfaceTexture` JNI jobject
  // TODO Add support for other platforms.
  TexturePtr CreateExternalTexture(
      void* native_stream, int2 size = {1, 1},
      ContentSecurityLevel security_level = ContentSecurityLevel::kNone);

  TexturePtr CreateExternalTexture(
      intptr_t texture_id, int2 size = {1, 1},
      ContentSecurityLevel security_level = ContentSecurityLevel::kNone);

  TexturePtr CreateExternalTexture(
      int2 size = {1, 1},
      ContentSecurityLevel security_level = ContentSecurityLevel::kNone);

  OwnedTexturePtr CreateExternalTexture(ExternalImageHandle handle,
                                        TextureCreationSettings settings);

  // Loads a texture from the cached texture asset.
  OwnedTexturePtr CreateTexture(AssetPtr<TextureAsset> texture,
                                TextureSamplerOptions options = {});

  // Loads a texture from the given image asset.
  //
  // Note: this is currently the only variant that can be used via Split Engine.
  OwnedTexturePtr CreateTexture(
      AssetPtr<ImageAsset> image,
      TextureGenerationOptions generation_options = {},
      TextureSamplerOptions sampler_options = {});



  // Creates an empty texture of specified size, levels, format, sampler type,
  // usage, options and native texture id.
  OwnedTexturePtr CreateTexture(TextureCreationSettings settings);

  // Creates a 2d texture array by passing in a span of image assets.
  // Returns a 3d texture that contains layers of 2d textures.
  // Dimensions of the 3d texture will be as follows:
  // x: Maximum width of the textures.
  // y: Maximum height of the textures.
  // z: Number of the textures.
  // The textures' upper left will always be at (0, 0).
  // Here's a minimal shader that shows how to sample the texture array:
  // third_party/impress/core/view/framewoxrk/tests/data/texture_array.mat

  TexturePtr CreateTexture(absl::Span<const AssetPtr<ImageAsset>> images,
                           TextureGenerationOptions generation_options,
                           TextureSamplerOptions sampler_options);
  TexturePtr CreateTexture(absl::Span<const AssetPtr<ImageAsset>> images);

  // Creates a texture from the given image contents.
  //
  // Although this returns immediately, the texture data is uploaded
  // asynchronously.
  // TODO: This variant correctly sends the texture data to split
  // engine. The other variants should be updated to do the same.

  OwnedTexturePtr CreateTexture(
      image::ImageContents& contents,
      TextureGenerationOptions generation_options,
      TextureSamplerOptions sampler_options,
      std::optional<absl::string_view> name = std::nullopt);

  // Creates a 2D texture with mipmaps from the given image assets
  // if generated_mipmap_levels in generation_options are not set. The first
  // image in the span is the base image, and the rest are mipmaps are in the
  // decreasing resolution order of mipmap levels.
  OwnedTexturePtr CreateTextureWithMipmaps(
      absl::Span<const AssetPtr<ImageAsset>> images,
      TextureGenerationOptions generation_options,
      TextureSamplerOptions sampler_options);

  // Wraps a filament::Texture with an imp::TexturePtr. As imp::TexturePtr is a
  // unique_ptr, this makes it the official owner of the memory.
  TexturePtr WrapTexture(filament::Texture* texture,
                         const filament::backend::SamplerParams& params = {});

  // Borrows a placeholder texture matching the target and internal format of
  // the given texture.
  absl::StatusOr<BorrowedTexturePtr> BorrowMatchingPlaceholderTexture(
      const BorrowedTexturePtr& texture,
      SmallSourceLocation loc = SmallSourceLocation::Current());

  // Borrows a white placeholder texture for assigning to unused texture
  // samplers.
  BorrowedTexturePtr BorrowPlaceholderTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  // Borrows a black placeholder texture for assigning to unused texture
  // samplers.
  BorrowedTexturePtr BorrowPlaceholderTextureBlack(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  // Borrows a placeholder cubemap texture for assigning to unused texture
  // samplers.
  BorrowedTexturePtr BorrowPlaceholderCubemapTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  // Borrows a placeholder texture for assigning to unused texture samplers.
  BorrowedTexturePtr BorrowRGBA32FPlaceholderTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  // Borrows a placeholder texture for assigning to unused texture samplers.
  BorrowedTexturePtr BorrowRGBA32UIPlaceholderTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  // Borrows a placeholder texture for assigning to unused texture samplers.
  BorrowedTexturePtr BorrowR11G11B10FPlaceholderTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  BorrowedTexturePtr BorrowRGBA8ArrayPlaceholderTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  BorrowedTexturePtr BorrowRGBA32UIArrayPlaceholderTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  BorrowedTexturePtr BorrowR11G11B10FArrayPlaceholderTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

 private:
  // Creates a singleton placeholder to borrow via BorrowPlaceholderTexture.
  OwnedTexturePtr CreateRGBA8PlaceholderTexture(uint32_t pixel);
  // Creates a singleton placeholder 2D texture array to borrow.
  OwnedTexturePtr CreateRGBA8ArrayPlaceholderTexture();
  // Creates a singleton RGBA32UI placeholder 2D texture array to borrow.
  OwnedTexturePtr CreateRGBA32UIArrayPlaceholderTexture();
  // Creates a singleton placeholder cubemap to borrow via
  // BorrowPlaceholderCubemapTexture().
  OwnedTexturePtr CreatePlaceholderCubemapTexture();
  // Creates a singleton RGBA32F placeholder to borrow.
  OwnedTexturePtr CreateRGBA32FPlaceholderTexture();
  // Creates a singleton RGBA32UI placeholder to borrow.
  OwnedTexturePtr CreateRGBA32UIPlaceholderTexture();
  // Creates a singleton R11G11B10F placeholder to borrow.
  OwnedTexturePtr CreateR11G11B10FPlaceholderTexture();
  // Creates a singleton R11G11B10F placeholder 2D texture array to borrow.
  OwnedTexturePtr CreateR11G11B10FArrayPlaceholderTexture();

  // Creates a 2x2 placeholder texture with the given pixel
  // value, format, data format, and data type.
  template <typename T>
  OwnedTexturePtr Create2x2PlaceholderTexture(
      T pixel_value, filament::backend::TextureFormat format,
      filament::backend::PixelDataFormat data_format,
      filament::backend::PixelDataType data_type,
      TextureSamplerOptions sampler_options, absl::string_view name) {
    constexpr int kSize = 2;
    auto pixels = std::make_unique<std::vector<T>>(4, pixel_value);
    image::OwnedImageContents<T> image_contents(kSize, kSize, std::move(pixels),
                                                format, data_format, data_type);
    return CreateTexture(image_contents, TextureGenerationOptions{},
                         sampler_options, name);
  }

  BaseView& view_;
  OwnedTexturePtr rgba8_white_placeholder_texture_;
  OwnedTexturePtr rgba8_black_placeholder_texture_;
  OwnedTexturePtr placeholder_cubemap_texture_;
  OwnedTexturePtr rgba32f_placeholder_texture_;
  OwnedTexturePtr rgba32ui_placeholder_texture_;
  OwnedTexturePtr r11g11b10f_placeholder_texture_;
  OwnedTexturePtr rgba8_array_placeholder_texture_;
  OwnedTexturePtr rgba32ui_array_placeholder_texture_;
  OwnedTexturePtr r11g11b10f_array_placeholder_texture_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_FACTORY_H_
