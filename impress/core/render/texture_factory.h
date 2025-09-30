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

#include "absl/base/attributes.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/assets/asset_ptr.h"
#include "core/common/small_source_location.h"
#include "core/image/image_contents.h"
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

  // Options are parameters to change the appearance of textures created by the
  // TextureFactory.
  struct ABSL_DEPRECATED(
      "Use TextureGenerationOptions and TextureSamplerOptions instead.")
      Options {
    // Determines how texture coordinates outside of [0,1] are handled.
    WrapMode wrap_mode = WrapMode::CLAMP_TO_EDGE;

    // Sampling method used when a texel covers multiple pixels.
    MagFilter mag_filter = MagFilter::LINEAR;

    // Sampling method used when a pixel covers multiple texels.
    MinFilter min_filter = MinFilter::LINEAR;

    // Adds extra texture samples to improve textures which are displayed at an
    // oblique angle to the camera. Should be a power-of-two. The default is 0.
    // The maximum permissible value is 7.
    float anisotropy = 0;

    // Optionally specify the number of mipmap levels to generate at runtime.
    // If no value is given, mipmaps are not generated.
    std::optional<uint8_t> generated_mipmap_levels = {};

    // Optional override to the default format in the ImageAsset object.
    std::optional<Format> texture_format_override = {};
  };

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
    // Sampler type of the texture.
    std::optional<SamplerType> sampler_type;
    // Usage of the texture.
    std::optional<Usage> usage;
    // With Metal, the id<MTLTexture> object should be cast to an intptr_t
    // using CFBridgingRetain to transfer ownership to Filament. Filament
    // will release ownership of the texture object when the Filament texture
    // is destroyed. The width, height, levels, and format should match what the
    // metal texture returns.
    std::optional<intptr_t> native_texture_id;
    // Other options of the texture. See TextureFactory::Options for details.
    ABSL_DEPRECATED("Use TextureCreationSettings::sampler_options instead.")
    std::optional<Options> options;
    // Sampler options for the texture.
    std::optional<TextureSamplerOptions> sampler_options;
  };

  TextureFactory(BaseView& view);

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
      filament::Stream* stream, int2 size = {1, 1},
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
  OwnedTexturePtr CreateTexture(AssetPtr<TextureAsset> texture);
  OwnedTexturePtr CreateTexture(AssetPtr<TextureAsset> texture,
                                TextureSamplerOptions options);

  // Create a 'normal' texture from the resource.
  // TODO: Refactor to use AssetPtr<ImageAsset>
  TexturePtr CreateTexture(const ImageAsset& image);

  // Create a texture and specify options.
  ABSL_DEPRECATED(
      "Use CreateTexture(..., TextureGenerationOptions, TextureSamplerOptions) "
      "instead.")
  TexturePtr CreateTexture(const ImageAsset& image, Options options);

  // Create a texture and specify options.
  TexturePtr CreateTexture(const ImageAsset& image,
                           TextureGenerationOptions generation_options,
                           TextureSamplerOptions sampler_options);

  // Creates an empty texture of specified size and format.
  // TODO: This does not send the texture data to split engine.
  TexturePtr CreateTexture(
      int width, int height, Format format,
      std::optional<absl::string_view> name = std::nullopt);

  // Creates an empty texture of specified size, format, and usage.
  TexturePtr CreateTexture(int width, int height, Format format, Usage usage);

  // Creates an empty texture of specified size, format, usage, and options.
  ABSL_DEPRECATED("Use CreateTexture(..., TextureSamplerOptions) instead.")
  TexturePtr CreateTexture(
      int width, int height, Format format, Usage usage, Options options,
      std::optional<absl::string_view> name = std::nullopt);

  // Creates an empty texture of specified size, format, usage, and options.
  TexturePtr CreateTexture(
      int width, int height, Format format, Usage usage,
      TextureSamplerOptions sampler_options,
      std::optional<absl::string_view> name = std::nullopt);

  // Creates a Filament texture by importing a native texture.
  // With Metal, the id<MTLTexture> object should be cast to an intptr_t
  // using CFBridgingRetain to transfer ownership to Filament. Filament
  // will release ownership of the texture object when the Filament texture
  // is destroyed. The width, height, levels, and format should match what the
  // metal texture returns.
  TexturePtr CreateTexture(intptr_t id, uint32_t width, uint32_t height,
                           uint8_t levels, Format format,
                           Usage usage = Usage::DEFAULT);

  // Creates an empty texture of specified size, levels, format, sampler type,
  // usage, options and native texture id.
  TexturePtr CreateTexture(TextureCreationSettings settings);

  // Creates a 2d texture array by passing in a span of image assets.
  // Returns a 3d texture that contains layers of 2d textures.
  // Dimensions of the 3d texture will be as follows:
  // x: Maximum width of the textures.
  // y: Maximum height of the textures.
  // z: Number of the textures.
  // The textures' upper left will always be at (0, 0).
  // Here's a minimal shader that shows how to sample the texture array:
  // third_party/impress/core/view/framewoxrk/tests/data/texture_array.mat
  ABSL_DEPRECATED(
      "Use CreateTexture(..., TextureGenerationOptions, TextureSamplerOptions) "
      "instead.")
  TexturePtr CreateTexture(absl::Span<const AssetPtr<ImageAsset>> images,
                           Options options);
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
  ABSL_DEPRECATED(
      "Use CreateTexture(..., TextureGenerationOptions, TextureSamplerOptions) "
      "instead.")
  OwnedTexturePtr CreateTexture(
      image::ImageContents& contents, Options options,
      std::optional<absl::string_view> name = std::nullopt);

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

  // Borrows a placeholder texture for assigning to unused texture samplers.
  BorrowedTexturePtr BorrowPlaceholderTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  // Borrows a placeholder cubemap texture for assigning to unused texture
  // samplers.
  BorrowedTexturePtr BorrowPlaceholderCubemapTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

 private:
  // Creates a singleton placeholder to borrow via BorrowPlaceholderTexture().
  OwnedTexturePtr CreatePlaceholderTexture();
  // Creates a singleton placeholder cubemap to borrow via
  // BorrowPlaceholderCubemapTexture().
  OwnedTexturePtr CreatePlaceholderCubemapTexture();

  BaseView& view_;
  OwnedTexturePtr placeholder_texture_;
  OwnedTexturePtr placeholder_cubemap_texture_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_TEXTURE_FACTORY_H_
