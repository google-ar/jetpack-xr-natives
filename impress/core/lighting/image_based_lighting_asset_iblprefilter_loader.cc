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

#include "core/lighting/image_based_lighting_asset_iblprefilter_loader.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/libs/iblprefilter/include/filament-iblprefilter/IBLPrefilterContext.h"
#include "filament/libs/image/include/image/LinearImage.h"
#include "filament/libs/imageio/include/imageio/ImageDecoder.h"
#include "core/async/executor.h"
#include "core/common/buffer_access.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"

namespace imp {

IblPrefilterLoader::IblPrefilterLoader(std::optional<int2> output_texture_size)
    : output_texture_size_(output_texture_size) {}

Future<std::unique_ptr<ImageBasedLightingAsset>> IblPrefilterLoader::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future) {
  return resource_future
      .Then(
          [url = std::string(asset_url)](resources::Resource resource)
              -> absl::StatusOr<std::unique_ptr<::image::LinearImage>> {
            BufferAccess encoded_image = resource.GetData();
            std::string encoded_image_string(encoded_image.StringView());
            std::istringstream in_stream(encoded_image_string);

            // Decode the HDR image with filament ImageDecoder.
            ::image::LinearImage image =
                ::image::ImageDecoder::decode(in_stream, url);
            if (image.getPixelRef() == nullptr) {
              return absl::InternalError("Failed to decode image.");
            }
            return std::make_unique<::image::LinearImage>(std::move(image));
          },
          Executor::Type::kBackground)
      .Then([output_texture_size = output_texture_size_, view,
             url = std::string(asset_url)](
                std::unique_ptr<::image::LinearImage> image)
                -> std::unique_ptr<ImageBasedLightingAsset> {
        uint32_t width = image->getWidth();
        uint32_t height = image->getHeight();

        // Create a texture from the decoded image.
        filament::Texture::PixelBufferDescriptor buffer(
            image->getPixelRef(),
            width * height * image->getChannels() * sizeof(float),
            filament::Texture::Format::RGB, filament::Texture::Type::FLOAT,
            [](void* buf, size_t, void* data) {
              // Called after filament finishes uploading the data to the GPU.
              // Converts the image back into a unique_ptr to destroy it.
              std::unique_ptr<::image::LinearImage> image(
                  reinterpret_cast<::image::LinearImage*>(data));
            },
            // Release the image, so that it isn't destroyed until it's finished
            // being uploaded to the gpu.
            image.release());

        filament::Texture* equirect_texture =
            filament::Texture::Builder()
                .width(width)
                .height(height)
                .levels(0xff)
                .format(filament::Texture::InternalFormat::R11F_G11F_B10F)
                .sampler(filament::Texture::Sampler::SAMPLER_2D)
                .build(*view->GetSharedEngine());

        equirect_texture->setImage(*view->GetSharedEngine(), 0,
                                   std::move(buffer));

        // If there is a custom output size, create a custom output texture.
        filament::Texture* skybox_texture = nullptr;
        if (output_texture_size.has_value()) {
          skybox_texture =
              filament::Texture::Builder()
                  .sampler(filament::Texture::Sampler::SAMPLER_CUBEMAP)
                  .format(filament::Texture::InternalFormat::R11F_G11F_B10F)
                  .usage(filament::Texture::Usage::COLOR_ATTACHMENT |
                         filament::Texture::Usage::SAMPLEABLE)
                  .width(output_texture_size->x)
                  .height(output_texture_size->y)
                  .levels(0xFF)
                  .build(*view->GetSharedEngine());
        }

        // Convert the texture into cubemaps with IblPrefilter.
        ::IBLPrefilterContext context(*view->GetSharedEngine());
        ::IBLPrefilterContext::EquirectangularToCubemap
            equirectangularToCubemap(context);
        ::IBLPrefilterContext::SpecularFilter specularFilter(context);

        // If skybox texture is nullptr, then filament creates a default one so
        // we assign that to the skybox texture. If the skybox texture is
        // assigned, then it just gets returned making the assignment here a
        // no-op.
        skybox_texture =
            equirectangularToCubemap(equirect_texture, skybox_texture);

        filament::Texture* ibl_texture = specularFilter(skybox_texture);

        view->GetSharedEngine()->destroy(equirect_texture);

        return std::make_unique<ImageBasedLightingAsset>(
            nullptr,
            OwnedTexturePtr(view->GetTextureFactory().WrapTexture(ibl_texture)),
            OwnedTexturePtr(
                view->GetTextureFactory().WrapTexture(skybox_texture)),
            std::string(url));
      });
}

}  // namespace imp
