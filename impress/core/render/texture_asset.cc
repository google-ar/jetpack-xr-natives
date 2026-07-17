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

#include "core/render/texture_asset.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "core/common/log.h"
#include "core/async/executor.h"
#if IMP_PLATFORM(WASM)
#include <cstdint>
#endif

#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/common/file_helpers.h"
#include "core/config.h"
#include "core/image/decode_image.h"
#include "core/image/image_contents.h"
#include "core/render/image_helpers.h"
#if IMP_PLATFORM(WASM)
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "core/image/wasm_decode_image.h"
#include "core/image/wasm_texture_contents.h"
#endif
#include "core/render/texture_builder.h"
#include "core/render/texture_options.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp {

Future<std::unique_ptr<TextureAsset>> TextureAsset::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future,
    TextureGenerationOptions options) {
  if (!Executor::IsOnForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "TextureAsset::Load must be called on the foreground thread.";
  }

  return resource_future.Then(
      [ctx = view->GetContext(), asset_url_copy = std::string(asset_url),
       options, view](resources::Resource resource)
          -> Future<std::unique_ptr<TextureAsset>> {
#if IMP_PLATFORM(WASM)
        filament::backend::TextureFormat format =
            options.texture_format_override.value_or(
                filament::backend::TextureFormat::SRGB8_A8);
        uint8_t requested_levels = 1;
        if (options.generated_mipmap_levels.has_value()) {
          requested_levels = options.generated_mipmap_levels.value();
        } else if (options.generate_mipmaps) {
          // We want to generate all levels, but since we don't know the
          // dimensions of the image yet we leave the calculation to after the
          // image has been decoded.
          requested_levels = 0xff;
        }
        return image::details::WasmDecodeImageToTexture(
                   asset_url_copy, resource, format, requested_levels)
            .Then([view, asset_url_copy,
                   options](WasmTextureContents texture_contents)
                      -> std::unique_ptr<TextureAsset> {
              return std::make_unique<TextureAsset>(
                  view, asset_url_copy, std::move(texture_contents), options);
            });
#else
        return image::DecodeImage(ctx, asset_url_copy, resource)
            .Then([view, asset_url_copy, options](
                      std::unique_ptr<image::ImageContents> image_contents)
                      -> std::unique_ptr<TextureAsset> {
              return std::make_unique<TextureAsset>(
                  view, asset_url_copy, std::move(image_contents), options);
            });
#endif
      },
#if IMP_PLATFORM(WASM)
      // On Wasm, the WebGL context is not available on background threads.
      // Actual decoding is offloaded to the browser in JS.
      Executor::Type::kForeground
#else
      Executor::Type::kBackground
#endif
  );
}

TextureAsset::TextureAsset(BaseView* view, absl::string_view texture_name,
                           std::unique_ptr<image::ImageContents> image_contents,
                           TextureGenerationOptions options)
    : view_(view), texture_name_(texture_name) {
  if (!Executor::IsOnForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "Texture asset needs to be created on the main thread.";
  }

  filament::Engine* engine = view_->GetSharedEngine();

  TextureBuilder texture_builder(*view_);
  if (!texture_name.empty()) {
    texture_builder.Name(
        absl::StrFormat("%s_tex", GetLocalFilenameFromFilename(texture_name)));
  }
  texture_builder.Sampler(filament::Texture::Sampler::SAMPLER_2D);
  texture_builder.Format(options.texture_format_override.value_or(
      image_contents->GetTextureFormat()));
  texture_builder.Width(image_contents->GetWidth());
  texture_builder.Height(image_contents->GetHeight());
  uint8_t levels = 1;
  if (options.generated_mipmap_levels.has_value()) {
    levels = std::min(options.generated_mipmap_levels.value(),
                      GetMipmapLevelCount(image_contents->GetWidth(),
                                          image_contents->GetHeight()));
  } else if (options.generate_mipmaps) {
    levels = GetMipmapLevelCount(image_contents->GetWidth(),
                                 image_contents->GetHeight());
  }
  texture_builder.Levels(levels);
  texture_builder.Image(*engine, *image_contents, {});
  if (levels > 1) {
    texture_builder.GenerateMipmaps(*engine);
  }
  texture_ = texture_builder.Build(*engine);
}

#if IMP_PLATFORM(WASM)
TextureAsset::TextureAsset(BaseView* view, absl::string_view texture_name,
                           WasmTextureContents texture_contents,
                           TextureGenerationOptions options)
    : view_(view),
      texture_name_(texture_name),
      gl_texture_id_(texture_contents.GetTextureId()) {
  if (!Executor::IsOnForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "Texture asset needs to be created on the main thread.";
  }

  if (!texture_name.empty()) {
    texture_name_ =
        absl::StrFormat("%s_tex", GetLocalFilenameFromFilename(texture_name));
  }
  filament::Engine* engine = view_->GetSharedEngine();

  TextureBuilder texture_builder(*view_);
  if (!texture_name.empty()) {
    texture_builder.Name(texture_name_);
  }

  filament::Texture::InternalFormat format =
      options.texture_format_override.value_or(
          filament::Texture::InternalFormat::SRGB8_A8);
  uint8_t levels = 1;
  if (options.generated_mipmap_levels.has_value()) {
    levels = std::min(options.generated_mipmap_levels.value(),
                      GetMipmapLevelCount(texture_contents.GetWidth(),
                                          texture_contents.GetHeight()));
  } else if (options.generate_mipmaps) {
    // We use the dimensions of the image to estimate the number of mip levels.
    levels = GetMipmapLevelCount(texture_contents.GetWidth(),
                                 texture_contents.GetHeight());
  }

  texture_ = texture_builder.Width(texture_contents.GetWidth())
                 .Height(texture_contents.GetHeight())
                 .Levels(levels)
                 .Format(format)
                 .Sampler(filament::Texture::Sampler::SAMPLER_2D)
                 .Import(texture_contents.GetTextureId())
                 .Usage(filament::Texture::Usage::DEFAULT)
                 .Build(*engine);
  if (levels > 1) {
    texture_->generateMipmaps(*engine);
  }
}
#endif

TextureAsset::~TextureAsset() {
  if (!Executor::IsOnForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "Texture asset needs to be destroyed on the main thread.";
  }
  if (view_ != nullptr && view_->GetSharedEngine() != nullptr &&
      texture_ != nullptr) {
    view_->GetSharedEngine()->destroy(texture_);
    texture_ = nullptr;
  }
#if IMP_PLATFORM(WASM)
  if (gl_texture_id_ != 0) {
    glDeleteTextures(1, &gl_texture_id_);
    gl_texture_id_ = 0;
  }
#endif
}

filament::Texture* TextureAsset::ReleaseFilamentTexture() {
  filament::Texture* texture = texture_;
  texture_ = nullptr;
  return texture;
}

}  // namespace imp
