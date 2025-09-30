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

#include "core/render/texture_factory.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/Platform.h"
#include "filament/filament/include/filament/Stream.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/assets/asset_ptr.h"
#include "core/common/file_helpers.h"
#include "core/common/small_source_location.h"
#include "core/image/image_contents.h"
#include "core/image/inline_image_contents.h"
#include "core/math/vec.h"
#include "core/render/content_security_level.h"
#include "core/render/image_asset.h"
#include "core/render/safe_filament_texture_builder.h"
#include "core/render/texture.h"
#include "core/render/texture_asset.h"
#include "core/render/texture_builder.h"
#include "core/render/texture_options.h"
#include "core/view/base_view.h"

namespace imp {

TextureFactory::TextureFactory(BaseView& view) : view_(view) {
  assert(view_.GetSharedEngine() != nullptr);
}

TexturePtr TextureFactory::CreateExternalTexture(
    int2 size, ContentSecurityLevel security_level) {
  filament::Engine* engine = view_.GetSharedEngine();

  auto texture_builder =
      filament::Texture::Builder()
          .levels(1)
          .width(size.x)
          .height(size.y)
          .sampler(filament::Texture::Sampler::SAMPLER_EXTERNAL);

  if (security_level == ContentSecurityLevel::kProtected) {
    if (!filament::Texture::isProtectedTexturesSupported(*engine)) {
      IMP_LOG(imp::ERROR) << "Protected textures are not supported on this backend.";
      return {};
    }
    texture_builder.usage(Usage::DEFAULT | Usage::PROTECTED);
  }

  filament::Texture* texture = texture_builder.build(*engine);
  if (!texture) {
    IMP_LOG(imp::ERROR) << "Could not create external texture";
    return {};
  }
  auto sampler =
      filament::TextureSampler(MagFilter::LINEAR, WrapMode::CLAMP_TO_EDGE);

  return absl::WrapUnique(
      new Texture(view_, nullptr, texture, sampler, security_level));
}

TexturePtr TextureFactory::CreateExternalTexture(
    void* native_stream, int2 size, ContentSecurityLevel security_level) {
  filament::Stream::Builder stream_builder;
  stream_builder.stream(native_stream);
  filament::Stream* stream = stream_builder.build(*view_.GetSharedEngine());
  if (stream == nullptr) {
    IMP_LOG(imp::ERROR) << "Could not create stream.";
    return {};
  }

  return CreateExternalTexture(stream, size, security_level);
}

TexturePtr TextureFactory::CreateExternalTexture(
    filament::Stream* stream, int2 size, ContentSecurityLevel security_level) {
  filament::Engine* engine = view_.GetSharedEngine();
  auto texture_builder =
      filament::Texture::Builder()
          .width(size.x)
          .height(size.y)
          .format(TextureFactory::Format::RGB8)
          .sampler(filament::Texture::Sampler::SAMPLER_EXTERNAL);

  if (security_level == ContentSecurityLevel::kProtected) {
    if (!filament::Texture::isProtectedTexturesSupported(*engine)) {
      IMP_LOG(imp::ERROR) << "Protected textures are not supported on this backend.";
      return {};
    }
    texture_builder.usage(Usage::DEFAULT | Usage::PROTECTED);
  }

  filament::Texture* texture = texture_builder.build(*engine);
  if (texture == nullptr) {
    IMP_LOG(imp::ERROR) << "Could not create external texture.";
    engine->destroy(stream);
    return {};
  }
  texture->setExternalStream(*engine, stream);

  filament::TextureSampler sampler(MagFilter::LINEAR, WrapMode::CLAMP_TO_EDGE);

  // Using `new` to access a non-public constructor, see (broken link).
  return absl::WrapUnique(
      new Texture(view_, stream, texture, sampler, security_level));
}

TexturePtr TextureFactory::CreateExternalTexture(
    intptr_t texture_id, int2 size, ContentSecurityLevel security_level) {
  filament::Engine* engine = view_.GetSharedEngine();
  auto texture_builder =
      filament::Texture::Builder()
          .sampler(filament::Texture::Sampler::SAMPLER_EXTERNAL)
          .format(TextureFactory::Format::RGB8)
          .import(texture_id)
          .width(size.x)
          .height(size.y);

  if (security_level == ContentSecurityLevel::kProtected) {
    if (!filament::Texture::isProtectedTexturesSupported(*engine)) {
      IMP_LOG(imp::ERROR) << "Protected textures are not supported on this backend.";
      return {};
    }
    texture_builder.usage(Usage::DEFAULT | Usage::PROTECTED);
  }

  filament::Texture* texture = texture_builder.build(*engine);
  if (texture == nullptr) {
    IMP_LOG(imp::ERROR) << "Could not create external texture.";
    return {};
  }

  filament::TextureSampler sampler(MagFilter::LINEAR, WrapMode::CLAMP_TO_EDGE);

  // Using `new` to access a non-public constructor, see (broken link).
  return absl::WrapUnique(
      new Texture(view_, nullptr, texture, sampler, security_level));
}

OwnedTexturePtr TextureFactory::CreateExternalTexture(
    filament::backend::Platform::ExternalImageHandle handle,
    TextureCreationSettings settings) {
  filament::Engine* engine = view_.GetSharedEngine();
  filament::Texture::Builder texture_builder = filament::Texture::Builder{}
                                                   .width(settings.width)
                                                   .height(settings.height)
                                                   .format(settings.format)
                                                   .external();
  if (settings.sampler_type) {
    texture_builder.sampler(*settings.sampler_type);
  }
  if (settings.usage) {
    texture_builder.usage(*settings.usage);
  }
  filament::Texture* texture = texture_builder.build(*view_.GetSharedEngine());
  if (!texture) {
    IMP_LOG(imp::ERROR) << "Could not create texture";
    return {};
  }
  texture->setExternalImage(*engine, handle);
  filament::TextureSampler sampler(MagFilter::LINEAR, WrapMode::CLAMP_TO_EDGE);
  return absl::WrapUnique(
      new Texture(view_, /*stream=*/nullptr, texture, sampler));
}

OwnedTexturePtr TextureFactory::CreateTexture(
    const AssetPtr<TextureAsset> texture) {
  return CreateTexture(texture, TextureSamplerOptions{});
}

OwnedTexturePtr TextureFactory::CreateTexture(
    const AssetPtr<TextureAsset> texture, TextureSamplerOptions options) {
  filament::TextureSampler sampler(options.mag_filter, options.wrap_mode);

  return absl::WrapUnique(
      new Texture(view_, /*stream=*/nullptr, texture, sampler));
}

TexturePtr TextureFactory::CreateTexture(const ImageAsset& image,
                                         Options options) {
  return CreateTexture(
      image,
      TextureGenerationOptions{
          .generated_mipmap_levels = options.generated_mipmap_levels,
          .texture_format_override = options.texture_format_override,
      },
      TextureSamplerOptions{
          .wrap_mode = options.wrap_mode,
          .mag_filter = options.mag_filter,
          .min_filter = options.min_filter,
          .anisotropy = options.anisotropy,
      });
}

TexturePtr TextureFactory::CreateTexture(
    const ImageAsset& image, TextureGenerationOptions generation_options,
    TextureSamplerOptions sampler_options) {
  // TODO : This is identical to ImageContents variant, but
  // returns a TexturePtr instead of an OwnedTexturePtr.
  filament::Engine* engine = view_.GetSharedEngine();

  TextureBuilder texture_builder(view_);
  if (!image.GetName().empty()) {
    texture_builder.Name(absl::StrFormat(
        "%s_tex", GetLocalFilenameFromFilename(image.GetName())));
  }
  texture_builder.Sampler(filament::Texture::Sampler::SAMPLER_2D);
  texture_builder.Format(generation_options.texture_format_override.has_value()
                             ? *generation_options.texture_format_override
                             : image.GetTextureFormat());
  texture_builder.Width(image.GetWidth());
  texture_builder.Height(image.GetHeight());
  if (generation_options.generated_mipmap_levels.has_value()) {
    texture_builder.Levels(generation_options.generated_mipmap_levels.value());
  }
  image::ImageContents& image_contents = image.GetImageContents();
  texture_builder.Image(*engine, image_contents, {});
  if (generation_options.generated_mipmap_levels.has_value()) {
    texture_builder.GenerateMipmaps(*engine);
  }
  filament::Texture* texture = texture_builder.Build(*engine);
  if (!texture) {
    IMP_LOG(imp::ERROR) << "Could not create texture from image, name: \""
               << image.GetName() << "\"";
    return {};
  }

  filament::TextureSampler sampler(sampler_options.min_filter,
                                   sampler_options.mag_filter,
                                   sampler_options.wrap_mode);
  sampler.setAnisotropy(sampler_options.anisotropy);

  // Using `new` to access a non-public constructor, see (broken link).
  TexturePtr result =
      absl::WrapUnique(new Texture(view_, nullptr, texture, sampler));
  result->SetName(image.GetName());
  return result;
}

TexturePtr TextureFactory::CreateTexture(const ImageAsset& image) {
  return CreateTexture(image, TextureGenerationOptions{},
                       TextureSamplerOptions{});
}

TexturePtr TextureFactory::CreateTexture(
    int width, int height, TextureFactory::Format format,
    std::optional<absl::string_view> name) {
  std::string name_str =
      name.has_value() ? absl::StrFormat("%s_tex", *name) : "";

  TextureBuilder texture_builder(view_);
  texture_builder.Format(format);
  texture_builder.Width(width);
  texture_builder.Height(height);
  texture_builder.Levels(1u);
  texture_builder.Name(name_str);
  filament::Texture* texture = texture_builder.Build(*view_.GetSharedEngine());
  if (!texture) {
    IMP_LOG(imp::ERROR) << "Could not create texture, name: \"" << name_str << "\"";
    return {};
  }
  filament::TextureSampler sampler(MagFilter::LINEAR, WrapMode::CLAMP_TO_EDGE);

  return absl::WrapUnique(new Texture(view_, nullptr, texture, sampler));
}

TexturePtr TextureFactory::CreateTexture(int width, int height,
                                         TextureFactory::Format format,
                                         TextureFactory::Usage usage) {
  return CreateTexture(width, height, format, usage, TextureSamplerOptions{});
}

TexturePtr TextureFactory::CreateTexture(
    int width, int height, TextureFactory::Format format,
    TextureFactory::Usage usage, Options options,
    std::optional<absl::string_view> name) {
  return CreateTexture(width, height, format, usage,
                       TextureSamplerOptions{
                           .wrap_mode = options.wrap_mode,
                           .mag_filter = options.mag_filter,
                           .min_filter = options.min_filter,
                           .anisotropy = options.anisotropy,
                       },
                       name);
}

// Creates an empty texture of specified size, format, usage, and options.
TexturePtr TextureFactory::CreateTexture(
    int width, int height, TextureFactory::Format format,
    TextureFactory::Usage usage, TextureSamplerOptions sampler_options,
    std::optional<absl::string_view> name) {
  std::string name_str =
      name.has_value() ? absl::StrFormat("%s_tex", *name) : "";

  filament::Texture* texture = filament::Texture::Builder{}
                                   .format(format)
                                   .width(width)
                                   .height(height)
                                   .usage(usage)
                                   .name(name_str.data(), name_str.length())
                                   .levels(1u)
                                   .build(*view_.GetSharedEngine());

  filament::TextureSampler sampler(sampler_options.min_filter,
                                   sampler_options.mag_filter,
                                   sampler_options.wrap_mode);
  sampler.setAnisotropy(sampler_options.anisotropy);

  return absl::WrapUnique(new Texture(view_, nullptr, texture, sampler));
}

TexturePtr TextureFactory::CreateTexture(intptr_t id, uint32_t width,
                                         uint32_t height, uint8_t levels,
                                         TextureFactory::Format format,
                                         TextureFactory::Usage usage) {
  auto texture = filament::Texture::Builder{}
                     .width(width)
                     .height(height)
                     .levels(levels)
                     .format(format)
                     .sampler(filament::Texture::Sampler::SAMPLER_2D)
                     .import(id)
                     .usage(usage)
                     .build(*view_.GetSharedEngine());

  auto sampler =
      filament::TextureSampler(MagFilter::LINEAR, WrapMode::CLAMP_TO_EDGE);
  return absl::WrapUnique(new Texture(view_, nullptr, texture, sampler));
}

TexturePtr TextureFactory::CreateTexture(TextureCreationSettings settings) {
  // This place is reachable from Renderer, use SafeFilamentTextureBuilder to
  // prevent panics.
  SafeFilamentTextureBuilder texture_builder = SafeFilamentTextureBuilder{}
                                                   .width(settings.width)
                                                   .height(settings.height)
                                                   .format(settings.format);

  if (settings.depth) {
    texture_builder.depth(*settings.depth);
  }

  if (settings.usage) {
    texture_builder.usage(*settings.usage);
  }

  if (settings.levels) {
    texture_builder.levels(*settings.levels);
  }

  if (settings.native_texture_id) {
    texture_builder.import(*settings.native_texture_id);
  }

  if (settings.sampler_type) {
    texture_builder.sampler(*settings.sampler_type);
  }

  absl::StatusOr<filament::Texture*> texture =
      texture_builder.build(*view_.GetSharedEngine());
  if (!texture.ok()) {
    return {};
  }

  filament::TextureSampler sampler =
      filament::TextureSampler(MagFilter::LINEAR, WrapMode::CLAMP_TO_EDGE);

  if (settings.sampler_options) {
    sampler = filament::TextureSampler(settings.sampler_options->min_filter,
                                       settings.sampler_options->mag_filter,
                                       settings.sampler_options->wrap_mode);
    sampler.setAnisotropy(settings.sampler_options->anisotropy);
  } else if (settings.options) {
    sampler = filament::TextureSampler(settings.options->min_filter,
                                       settings.options->mag_filter,
                                       settings.options->wrap_mode);
    sampler.setAnisotropy(settings.options->anisotropy);
  }

  return absl::WrapUnique(new Texture(view_, nullptr, *texture, sampler));
}

TexturePtr TextureFactory::CreateTexture(
    absl::Span<const AssetPtr<ImageAsset>> images, Options options) {
  return CreateTexture(
      images,
      TextureGenerationOptions{
          .generated_mipmap_levels = options.generated_mipmap_levels,
          .texture_format_override = options.texture_format_override,
      },
      TextureSamplerOptions{
          .wrap_mode = options.wrap_mode,
          .mag_filter = options.mag_filter,
          .min_filter = options.min_filter,
          .anisotropy = options.anisotropy,
      });
}

TexturePtr TextureFactory::CreateTexture(
    absl::Span<const AssetPtr<ImageAsset>> images,
    TextureGenerationOptions generation_options,
    TextureSamplerOptions sampler_options) {
  if (images.empty()) {
    IMP_LOG(imp::ERROR) << "CreateTexture: image array cannot be empty.";
    return {};
  }

  // TODO: support texture arrays.
  if (view_.GetSplitEngineSerializer()) {
    IMP_LOG(imp::WARNING) << "Texture arrays are not supported in split-engine mode!";
  }

  filament::Engine* engine = view_.GetSharedEngine();

  uint3 texture_dimensions = {0, 0, images.size()};
  filament::backend::TextureFormat format = images[0]->GetTextureFormat();
  for (const AssetPtr<ImageAsset>& image : images) {
    texture_dimensions.x = std::max(image->GetWidth(), texture_dimensions.x);
    texture_dimensions.y = std::max(image->GetHeight(), texture_dimensions.y);
  }

  filament::Texture::Builder texture_builder;
  texture_builder.sampler(filament::Texture::Sampler::SAMPLER_2D_ARRAY);
  texture_builder.levels(
      generation_options.generated_mipmap_levels.value_or(1));
  texture_builder.format(format);
  texture_builder.width(texture_dimensions.x);
  texture_builder.height(texture_dimensions.y);
  texture_builder.depth(texture_dimensions.z);
  filament::Texture* texture = texture_builder.build(*engine);
  if (texture == nullptr) {
    IMP_LOG(imp::ERROR) << "Could not create texture.";
    return {};
  }

  for (int i = 0; i < images.size(); ++i) {
    const AssetPtr<ImageAsset>& image = images[i];
    auto descriptors = image->GetLevelDescriptors();
    for (int level = 0; level < descriptors.size(); ++level) {
      texture->setImage(*engine, level, 0, 0, i, texture->getWidth(level),
                        texture->getHeight(level), 1,
                        std::move(descriptors[level]));
    }
  }

  if (generation_options.generated_mipmap_levels.has_value()) {
    texture->generateMipmaps(*engine);
  }

  filament::TextureSampler sampler(sampler_options.min_filter,
                                   sampler_options.mag_filter,
                                   sampler_options.wrap_mode);
  sampler.setAnisotropy(sampler_options.anisotropy);

  return absl::WrapUnique(new Texture(view_, nullptr, texture, sampler));
}

TexturePtr TextureFactory::CreateTexture(
    absl::Span<const AssetPtr<ImageAsset>> images) {
  return CreateTexture(images, TextureGenerationOptions{},
                       TextureSamplerOptions{
                           .mag_filter = MagFilter::NEAREST,
                           .min_filter = MinFilter::NEAREST,
                       });
}

OwnedTexturePtr TextureFactory::CreateTexture(
    image::ImageContents& image_contents, Options options,
    std::optional<absl::string_view> name) {
  return CreateTexture(
      image_contents,
      TextureGenerationOptions{
          .generated_mipmap_levels = options.generated_mipmap_levels,
          .texture_format_override = options.texture_format_override,
      },
      TextureSamplerOptions{
          .wrap_mode = options.wrap_mode,
          .mag_filter = options.mag_filter,
          .min_filter = options.min_filter,
          .anisotropy = options.anisotropy,
      },
      name);
}

OwnedTexturePtr TextureFactory::CreateTexture(
    image::ImageContents& image_contents,
    TextureGenerationOptions generation_options,
    TextureSamplerOptions sampler_options,
    std::optional<absl::string_view> name) {
  // TODO : This is identical to ImageAsset variant, but
  // returns a OwnedTexturePtr instead of a TexturePtr.
  filament::Engine* engine = view_.GetSharedEngine();

  TextureBuilder texture_builder(view_);
  if (name.has_value()) {
    texture_builder.Name(
        absl::StrFormat("%s_tex", GetLocalFilenameFromFilename(*name)));
  }

  texture_builder.Sampler(filament::Texture::Sampler::SAMPLER_2D);
  texture_builder.Format(generation_options.texture_format_override.has_value()
                             ? *generation_options.texture_format_override
                             : image_contents.GetTextureFormat());

  texture_builder.Width(image_contents.GetWidth());
  texture_builder.Height(image_contents.GetHeight());
  if (generation_options.generated_mipmap_levels.has_value()) {
    texture_builder.Levels(generation_options.generated_mipmap_levels.value());
  } else {
    texture_builder.Levels(image_contents.GetLevelCount());
  }
  texture_builder.Image(*engine, image_contents, {});
  if (generation_options.generated_mipmap_levels.has_value()) {
    texture_builder.GenerateMipmaps(*engine);
  }
  filament::Texture* texture = texture_builder.Build(*engine);
  if (!texture) {
    IMP_LOG(imp::ERROR) << "Could not create texture.";
    return {};
  }

  filament::TextureSampler sampler(sampler_options.min_filter,
                                   sampler_options.mag_filter,
                                   sampler_options.wrap_mode);
  sampler.setAnisotropy(sampler_options.anisotropy);

  // Using `new` to access a non-public constructor, see (broken link).
  OwnedTexturePtr result = WrapTexture(texture);

  if (name.has_value()) {
    result->SetName(*name);
  }
  return result;
}

OwnedTexturePtr TextureFactory::CreateTextureWithMipmaps(
    absl::Span<const AssetPtr<ImageAsset>> images,
    TextureGenerationOptions generation_options,
    TextureSamplerOptions sampler_options) {
  if (images.empty()) {
    IMP_LOG(imp::ERROR) << "CreateTexture: image array cannot be empty.";
    return {};
  }

  filament::Engine* engine = view_.GetSharedEngine();

  const ImageAsset& base_image = *images[0];

  filament::Texture::Builder texture_builder;
  texture_builder.sampler(SamplerType::SAMPLER_2D);
  texture_builder.levels(
      generation_options.generated_mipmap_levels.value_or(images.size()));
  texture_builder.format(base_image.GetTextureFormat());
  texture_builder.width(base_image.GetWidth());
  texture_builder.height(base_image.GetHeight());
  texture_builder.depth(1);

  filament::Texture* texture = texture_builder.build(*engine);
  if (texture == nullptr) {
    IMP_LOG(imp::ERROR) << "Could not create texture.";
    return {};
  }

  if (generation_options.generated_mipmap_levels.has_value()) {
    texture->generateMipmaps(*engine);
  } else {
    for (int level = 0; level < images.size(); ++level) {
      auto descriptors = images[level]->GetLevelDescriptors();
      if (descriptors.empty()) {
        IMP_LOG(imp::ERROR) << "Failed to create texture: image level descriptors needs "
                      "to have at least one level.";
        return {};
      }

      if (images[level]->GetWidth() != texture->getWidth(level) ||
          images[level]->GetHeight() != texture->getHeight(level)) {
        IMP_LOG(imp::ERROR) << "Failed to create texture: image level dimensions do "
                      "not match.";
        return {};
      }

      texture->setImage(*engine, level, 0, 0, 0, texture->getWidth(level),
                        texture->getHeight(level), 1,
                        std::move(descriptors[0]));
    }
  }

  filament::TextureSampler sampler(sampler_options.min_filter,
                                   sampler_options.mag_filter,
                                   sampler_options.wrap_mode);
  sampler.setAnisotropy(sampler_options.anisotropy);

  return absl::WrapUnique(new Texture(view_, nullptr, texture, sampler));
}

TexturePtr TextureFactory::WrapTexture(
    filament::Texture* texture,
    const filament::backend::SamplerParams& params) {
  return absl::WrapUnique(
      new Texture(view_, nullptr, texture, filament::TextureSampler(params)));
}

BorrowedTexturePtr TextureFactory::BorrowPlaceholderTexture(
    SmallSourceLocation loc) {
  if (!placeholder_texture_) {
    placeholder_texture_ = CreatePlaceholderTexture();
  }
  return placeholder_texture_.Borrow(loc);
}

BorrowedTexturePtr TextureFactory::BorrowPlaceholderCubemapTexture(
    SmallSourceLocation loc) {
  if (!placeholder_cubemap_texture_) {
    placeholder_cubemap_texture_ = CreatePlaceholderCubemapTexture();
  }
  return placeholder_cubemap_texture_.Borrow(loc);
}

OwnedTexturePtr TextureFactory::CreatePlaceholderTexture() {
  constexpr uint32_t kPixel = 0xffffffff;
  constexpr int kPlaceholderTextureSize = 2;
  constexpr int kNumPlaceholderTexturePixels =
      kPlaceholderTextureSize * kPlaceholderTextureSize;

  // Pixels are static to ensure that the memory is not freed when the function
  // exits, it must live until the data is uploaded to the GPU.
  static constexpr std::array<uint32_t, kNumPlaceholderTexturePixels>
      kPlaceholderTexturePixels = {kPixel, kPixel, kPixel, kPixel};

  InlineImageContents image_contents(
      kPlaceholderTextureSize, kPlaceholderTextureSize,
      reinterpret_cast<const uint8_t*>(kPlaceholderTexturePixels.data()),
      (sizeof(uint32_t) * kNumPlaceholderTexturePixels),
      filament::Texture::InternalFormat::RGBA8);

  return CreateTexture(image_contents, TextureGenerationOptions{},
                       TextureSamplerOptions{}, "PlaceholderTexture");
}

OwnedTexturePtr TextureFactory::CreatePlaceholderCubemapTexture() {
  constexpr int kPlaceholderTextureSize = 1;
  constexpr int kPixelsPerFace =
      kPlaceholderTextureSize * kPlaceholderTextureSize;
  constexpr int kChannels = 4;
  constexpr int kBytesPerFace = kPixelsPerFace * kChannels;

  std::vector<uint8_t> memory(kBytesPerFace * 6, 0xff);

  image::StitchedImageContents image_contents(kPlaceholderTextureSize,
                                              kPlaceholderTextureSize * 6,
                                              kChannels, std::move(memory));

  OwnedTexturePtr result =
      CreateTexture(imp::TextureFactory::TextureCreationSettings{
          .width = kPlaceholderTextureSize,
          .height = kPlaceholderTextureSize,
          .format = filament::Texture::InternalFormat::RGBA8,
          .sampler_type = filament::Texture::Sampler::SAMPLER_CUBEMAP});

  filament::Engine* engine = view_.GetSharedEngine();
  filament::Texture::FaceOffsets face_offsets(kBytesPerFace);
  result->GetTexture()->setImage(
      *engine, /*level=*/0,
      image_contents.CreatePixelBufferDescriptor(
          /*callback=*/nullptr, /*is_r11_g11_b10=*/false),
      face_offsets);

  result->SetName("PlaceholderCubemapTexture");

  return result;
}

}  // namespace imp
