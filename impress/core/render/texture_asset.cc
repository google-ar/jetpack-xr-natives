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

#include <memory>
#include <string>
#include <utility>

#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/common/file_helpers.h"
#include "core/config.h"
#include "core/image/decode_image.h"
#include "core/image/image_contents.h"
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
  return resource_future.Then([view, asset_url_copy = std::string(asset_url),
                               options](resources::Resource resource)
                                  -> Future<std::unique_ptr<TextureAsset>> {
#if IMP_PLATFORM(WASM)
    return image::details::WasmDecodeImageToTexture(asset_url_copy, resource)
        .Then([view, asset_url_copy,
               options](WasmTextureContents texture_contents)
                  -> std::unique_ptr<TextureAsset> {
          return std::make_unique<TextureAsset>(
              view, asset_url_copy, std::move(texture_contents), options);
        });
#else
    return image::DecodeImage(view->GetContext(), asset_url_copy, resource)
        .Then([view, asset_url_copy,
               options](std::unique_ptr<image::ImageContents> image_contents)
                  -> std::unique_ptr<TextureAsset> {
          return std::make_unique<TextureAsset>(
              view, asset_url_copy, std::move(image_contents), options);
        });
#endif
  });
}

TextureAsset::TextureAsset(BaseView* view, absl::string_view texture_name,
                           std::unique_ptr<image::ImageContents> image_contents,
                           TextureGenerationOptions options)
    : view_(view), texture_name_(texture_name) {
  filament::Engine* engine = view_->GetSharedEngine();

  TextureBuilder texture_builder(*view_);
  if (!texture_name.empty()) {
    texture_builder.Name(
        absl::StrFormat("%s_tex", GetLocalFilenameFromFilename(texture_name)));
  }
  texture_builder.Sampler(filament::Texture::Sampler::SAMPLER_2D);
  texture_builder.Format(options.texture_format_override.has_value()
                             ? *options.texture_format_override
                             : image_contents->GetTextureFormat());
  texture_builder.Width(image_contents->GetWidth());
  texture_builder.Height(image_contents->GetHeight());
  if (options.generated_mipmap_levels.has_value()) {
    texture_builder.Levels(options.generated_mipmap_levels.value());
  }
  texture_builder.Image(*engine, *image_contents, {});
  if (options.generated_mipmap_levels.has_value()) {
    texture_builder.GenerateMipmaps(*engine);
  }
  texture_ = texture_builder.Build(*engine);
}

#if IMP_PLATFORM(WASM)
TextureAsset::TextureAsset(BaseView* view, absl::string_view texture_name,
                           WasmTextureContents texture_contents,
                           TextureGenerationOptions options)
    : view_(view), texture_name_(texture_name) {
  if (!texture_name.empty()) {
    texture_name_ =
        absl::StrFormat("%s_tex", GetLocalFilenameFromFilename(texture_name));
  }
  filament::Engine* engine = view_->GetSharedEngine();

  filament::Texture::Builder texture_builder{};
  if (!texture_name.empty()) {
    texture_builder =
        texture_builder.name(texture_name_.data(), texture_name.length());
  }
  texture_ = texture_builder.width(texture_contents.GetWidth())
                 .height(texture_contents.GetHeight())
                 .levels(options.generated_mipmap_levels.value_or(1))
                 .format(options.texture_format_override.value_or(
                     filament::backend::TextureFormat::SRGB8_A8))
                 .sampler(filament::Texture::Sampler::SAMPLER_2D)
                 .import(texture_contents.GetTextureId())
                 .usage(filament::Texture::Usage::DEFAULT)
                 .build(*engine);
  if (options.generated_mipmap_levels.has_value()) {
    texture_->generateMipmaps(*engine);
  }
}
#endif

TextureAsset::~TextureAsset() {
  if (view_ != nullptr && view_->GetSharedEngine() != nullptr &&
      texture_ != nullptr) {
    view_->GetSharedEngine()->destroy(texture_);
    texture_ = nullptr;
  }
}

}  // namespace imp
