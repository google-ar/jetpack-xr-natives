// Copyright 2025 Google LLC
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

#include "core/split_engine/texture_validator.h"

#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Texture.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

// TODO: Remove this copy, once we can use the Filament API
// Source: FTexture::maxLevelCount
// (broken link)
//
// Static method is not exposed to public API, so we need to duplicate it
// in order to replicate FILAMENT_PANIC_CHECK_PRECONDITION checks.
//
// Used to calculate the max number of levels to verify ImageParams->level().
static inline uint8_t MaxLevelCount(uint32_t maxDimension) {
  return std::max(1, std::ilogbf(static_cast<float>(maxDimension)) + 1);
}

// TODO: Remove this copy, once we can use the Filament API
// Source: FTexture::maxLevelCount
// (broken link)
//
// Static method is not exposed to public API, so we need to duplicate it
// in order to replicate FILAMENT_PANIC_CHECK_PRECONDITION checks.
//
// Used to calculate the max number of levels to verify ImageParams->level().
static inline uint8_t MaxLevelCount(uint32_t width, uint32_t height) {
  uint32_t const maxDimension = std::max(width, height);
  return MaxLevelCount(maxDimension);
}

static absl::Status ValidateSamplerType(
    filament::backend::SamplerType sampler_type) noexcept {
  switch (sampler_type) {
    case filament::backend::SamplerType::SAMPLER_2D:
    case filament::backend::SamplerType::SAMPLER_2D_ARRAY:
    case filament::backend::SamplerType::SAMPLER_CUBEMAP:
    case filament::backend::SamplerType::SAMPLER_EXTERNAL:
    case filament::backend::SamplerType::SAMPLER_3D:
    case filament::backend::SamplerType::SAMPLER_CUBEMAP_ARRAY:
      return absl::OkStatus();
  }

  // No default case to get a compiler error if new enum is added.
  // Return is reachable if user input is invalid.
  return absl::InvalidArgumentError(absl::StrFormat(
      "Invalid sampler type: %u", static_cast<uint32_t>(sampler_type)));
}

absl::Status TextureValidator::ValidateAddTexturesMessage(
    const android_xr::schemas::AddTextures& command, filament::Engine& engine) {
  if (!command.textures() || command.textures()->size() == 0) {
    return absl::InvalidArgumentError("Empty AddTextures message");
  }

  for (const android_xr::schemas::Texture* texture : *command.textures()) {
    if (texture->image_params()->size() != texture->pixel_buffers()->size()) {
      return absl::InvalidArgumentError(
          "Invalid AddTextures message. "
          "Arrays image_params and pixel_buffers must have the same length.");
    }

    // Prevent crash in Filament's Texture.cpp
    // FILAMENT_CHECK_PRECONDITION(validateSamplerType(mImpl->mTarget))
    MP_RETURN_IF_ERROR(ValidateSamplerType(
        static_cast<filament::backend::SamplerType>(texture->sampler())));

    // Prevent panic in Filament's VulkanTexture.cpp
    // assert_invariant(width <= this->width && height <= this->height);
    if (texture->width() == 0 || texture->height() == 0) {
      return absl::InvalidArgumentError(
          "Invalid AddTextures message. "
          "Texture width and height must be non-zero.");
    }

    if (texture->mips()) {
      // Prevent panic in Filament's FTexture::generateMipmaps:
      // FILAMENT_CHECK_PRECONDITION(mTarget != SamplerType::SAMPLER_3D)
      if (static_cast<filament::Texture::Sampler>(texture->sampler()) ==
          filament::backend::SamplerType::SAMPLER_3D) {
        return absl::InvalidArgumentError(
            "Invalid AddTextures message. "
            "3D Textures are not mipmappable.");
      }

      // Prevent panic in Filament's FTexture::generateMipmaps:
      // FILAMENT_CHECK_PRECONDITION(formatMipmappable)
      if (!filament::Texture::isTextureFormatMipmappable(
              engine, static_cast<filament::backend::TextureFormat>(
                          texture->format()))) {
        return absl::InvalidArgumentError(
            "Invalid AddTextures message. Texture format is not mipmappable.");
      }
    }

    // Prevent panic in Filament's Texture::Builder::build:
    // FILAMENT_CHECK_PRECONDITION(Texture::isTextureFormatSupported(engine,
    //     mImpl->mFormat))
    if (!filament::Texture::isTextureFormatSupported(
            engine,
            static_cast<filament::backend::TextureFormat>(texture->format()))) {
      return absl::InvalidArgumentError(
          "Invalid AddTextures message. Texture format is not supported.");
    }

    // Prevent panic in Filament's FTexture::setImage:
    // FILAMENT_CHECK_PRECONDITION(!mExternal)
    // mExternal is set to true when the sampler is SAMPLER_EXTERNAL.
    if (static_cast<filament::Texture::Sampler>(texture->sampler()) ==
        filament::backend::SamplerType::SAMPLER_EXTERNAL) {
      return absl::InvalidArgumentError(
          "Invalid AddTextures message. "
          "External textures are not supported.");
    }

    const auto texture_format =
        static_cast<filament::backend::TextureFormat>(texture->format());

    const int num_images = texture->image_params()->size();
    for (int i = 0; i < num_images; ++i) {
      const android_xr::schemas::PixelBuffer* pixel_buffer =
          texture->pixel_buffers()->Get(i);
      // Prevent two problems from happening:
      // 1. null pointer dereference
      // 2. Panic in Filmament's VulkanTexture.cpp:
      //    assert_invariant(hostData->size > 0 && "Data is empty");
      if (pixel_buffer->buffer() == nullptr ||
          pixel_buffer->buffer()->size() == 0) {
        return absl::InvalidArgumentError(
            "Invalid AddTextures message. "
            "Pixel buffer is either null or empty.");
      }

      const android_xr::schemas::ImageParams* image_params =
          texture->image_params()->Get(i);

      const auto pixel_data_type =
          static_cast<filament::backend::PixelDataType>(image_params->type());
      const auto pixel_data_format =
          static_cast<filament::backend::PixelDataFormat>(
              image_params->format());

      // Prevent panic in Filament's FTexture::setImage:
      // FILAMENT_CHECK_PRECONDITION(p.type == PixelDataType::COMPRESSED ||
      //     validatePixelFormatAndType(mFormat, p.format, p.type))
      if (pixel_data_type != filament::backend::PixelDataType::COMPRESSED &&
          !filament::Texture::validatePixelFormatAndType(
              texture_format, pixel_data_format, pixel_data_type)) {
        return absl::InvalidArgumentError(
            "Invalid AddTextures message. "
            "Invalid combination of format and type");
      }

      // Prevent panic in Filament's FTexture::setImage:
      // FILAMENT_CHECK_PRECONDITION(level < mLevelCount)
      // In the precondition:
      //  - level is image_params->level()
      //  - mLevelCount is the value calculated in TextureBuilder.

      // builder.levels() emulation
      const auto levels = [](uint8_t levels) {
        return std::max(static_cast<uint8_t>(1), levels);
      };
      // builder.build() emulation
      const auto build = [&texture](uint8_t levels) {
        return std::min(levels,
                        MaxLevelCount(texture->width(), texture->height()));
      };

      if (image_params->level() >= build(levels(texture->levels()))) {
        return absl::InvalidArgumentError(
            "Invalid AddTextures message. ImageParams has invalid level");
      }

      // Prevent panic in Filament's PBD::computeDataSize:
      // assert_invariant(alignment);
      if (image_params->alignment() == 0) {
        return absl::InvalidArgumentError(
            "Invalid AddTextures message. ImageParams has invalid alignment");
      }

      using PBD = filament::backend::PixelBufferDescriptor;
      const size_t stride =
          image_params->stride() ? image_params->stride() : texture->width();
      const size_t bpp =
          PBD::computeDataSize(pixel_data_format, pixel_data_type, 1, 1, 1);
      const size_t bpr =
          PBD::computeDataSize(pixel_data_format, pixel_data_type, stride, 1,
                               image_params->alignment());
      const size_t bpl = bpr * texture->height();
      // Prevent panic in Filament's FTexture::setImage:
      // FILAMENT_CHECK_PRECONDITION(bpp * p.left + bpr * p.top +
      //     bpl * (0 + depth) <= p.size)
      // NOTE: SplitEngineRendererImpl::AddTextures does not touch depth while
      // building the Filament texture, default value is 1.
      if (bpp * image_params->left() + bpr * image_params->top() +
              bpl * (0 + /* depth = */ 1) >
          pixel_buffer->buffer()->size()) {
        return absl::InvalidArgumentError(
            "Invalid AddTextures message. Pixel buffer is too small.");
      }
    }
  }

  return absl::OkStatus();
}

}  // namespace imp::split_engine
