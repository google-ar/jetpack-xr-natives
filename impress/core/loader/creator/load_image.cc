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

#include "core/loader/creator/load_image.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <variant>

#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/libs/ktxreader/include/ktxreader/Ktx1Reader.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/common/enum_flags.h"
#include "core/image/decode_image.h"
#include "core/image/image_contents.h"
#include "core/image/wrapped_image_contents.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/render/texture_asset.h"
#include "core/render/texture_options.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp::loader::details {

Future<std::variant<std::unique_ptr<image::ImageContents>,
                    std::unique_ptr<TextureAsset>>>
LoadImage(BaseView* view, const imp::Context& context,
          const schemas::TextureInfo* texture_info,
          schemas::ImageInfo image_info_type, const void* image_info,
          bool enable_use_texture_asset_api, std::function<void()> callback) {
  if (image_info_type == schemas::ImageInfo::CompressedImageData) {
    // Creates ImageContents from the compressed bytes stored in image_info.
    auto compressed_image_data =
        reinterpret_cast<const schemas::CompressedImageData*>(image_info);

    // Picks a compressed pixel data type from the texture format.
    auto compressed_pixel_data_type =
        ::ktxreader::Ktx1Reader::toCompressedFilamentEnum<
            filament::backend::CompressedPixelDataType>(
            compressed_image_data->format());

    return Future<std::variant<std::unique_ptr<image::ImageContents>,
                               std::unique_ptr<TextureAsset>>>(
        std::make_unique<image::WrappedImageContents>(
            compressed_image_data->width(), compressed_image_data->height(),
            static_cast<filament::backend::TextureFormat>(
                compressed_image_data->format()),
            compressed_pixel_data_type,
            absl::Span<const uint8_t>{
                compressed_image_data->buffer()->data(),
                compressed_image_data->buffer()->size()}));
  } else if (image_info_type == schemas::ImageInfo::ImageFileData) {
    // Decodes the PNG or JPEG data stored in image_info.
    // The caller of this function is responsible for keeping image_info alive
    // until the image is done decoding and possibly until the ImageContents is
    // no longer needed.
    auto image_file_data =
        reinterpret_cast<const schemas::ImageFileData*>(image_info);
    auto buffer_access = BufferAccess::Wrap(image_file_data->buffer()->Data(),
                                            image_file_data->buffer()->size());
    // The `callback` parameter is not actually a callback, but should rather be
    // thought of as a Holdable<LoaderInProgress>. The image decoder needs to
    // manage the lifetime of the underlying resource until it is finished
    // decoding the image, thus we destroy the callback in the cord releaser.
    resources::Resource resource(absl::MakeCordFromExternal(
        buffer_access.StringView(), [callback]() {}));

    if (view == nullptr || !enable_use_texture_asset_api) {
      return image::DecodeImage(context, texture_info->name()->c_str(),
                                resource)
          .Then([](std::unique_ptr<image::ImageContents> image_contents)
                    -> std::variant<std::unique_ptr<image::ImageContents>,
                                    std::unique_ptr<TextureAsset>> {
            return std::move(image_contents);
          });
    }

    TextureGenerationOptions options;
    const Flags<schemas::TextureInfoFlags> flags(texture_info->flags());

    if (flags.Test(schemas::TextureInfoFlags::IsR11G11B10)) {
      options.texture_format_override =
          filament::Texture::InternalFormat::R11F_G11F_B10F;
    } else if (flags.Test(schemas::TextureInfoFlags::IsSrgb)) {
      options.texture_format_override =
          filament::Texture::InternalFormat::SRGB8_A8;
    }

    if (flags.Test(schemas::TextureInfoFlags::GenerateMips) &&
        !flags.Test(schemas::TextureInfoFlags::IsLookup)) {
      options.generate_mipmaps = true;
    }

    Future<resources::Resource> resource_future;
    resource_future.Return(std::move(resource));

    return TextureAsset::Load(view, texture_info->name()->c_str(),
                              std::move(resource_future), options)
        .Then([](std::unique_ptr<TextureAsset> texture_asset)
                  -> std::variant<std::unique_ptr<image::ImageContents>,
                                  std::unique_ptr<TextureAsset>> {
          return std::move(texture_asset);
        });
  }

  return absl::InternalError("Invalid ImageInfo type");
}

}  // namespace imp::loader::details
