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

#include "core/image/wimage_decode_image.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>

#include "image/wimage/wimage.h"
#include "image/wimage/wimage_io_unsandboxed.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/background_delete.h"
#include "core/common/string_helpers.h"
#include "core/image/image_contents.h"
#include "core/resources/resource_manager.h"

namespace imp::image::details {
namespace {
static constexpr size_t kMaxDimension = 8192;

using ::filament::backend::PixelBufferDescriptor;

class WImageImageContents : public ImageContents {
 public:
  explicit WImageImageContents(WImageBuffer4_b image)
      : image_(imp::MakeSharedWithBackgroundDeleter<WImageBuffer4_b>(
            std::move(image))) {}

  uint32_t GetWidth() const override {
    return static_cast<uint32_t>(image_->Width());
  }
  uint32_t GetStride() const override {
    return static_cast<uint32_t>(image_->PixelSize() * image_->Width());
  }
  uint32_t GetHeight() const override {
    return static_cast<uint32_t>(image_->Height());
  }
  std::size_t GetSize() const override {
    return image_->Width() * image_->Height() * image_->PixelSize();
  }
  uint8_t* GetData() override { return image_->ImageData(); }
  bool HasAlpha() const override { return image_->Channels() == 4; }
  filament::backend::PixelBufferDescriptor CreatePixelBufferDescriptor(
      std::function<void()> callback, bool is_r11_g11_b10) override;
  filament::backend::TextureFormat GetTextureFormat() const override {
    return HasAlpha() ? filament::Texture::InternalFormat::SRGB8_A8
                      : filament::Texture::InternalFormat::SRGB8;
  }

 private:
  std::shared_ptr<WImageBuffer4_b> image_;
};

PixelBufferDescriptor WImageImageContents::CreatePixelBufferDescriptor(
    std::function<void()> callback, bool is_r11_g11_b10) {
  // Packet whose lifetime begins when a Texture's byte buffer data is queued
  // for consumption by the render thread, and ends when the data is consumed.
  struct TextureUpload {
    std::shared_ptr<WImageBuffer4_b> image;
    std::function<void()> callback;
  };
  TextureUpload* texture_upload = new TextureUpload{image_, callback};
  return PixelBufferDescriptor(
      texture_upload->image->ImageData(),
      texture_upload->image->Width() * texture_upload->image->Height() *
          texture_upload->image->PixelSize(),
      (!HasAlpha() || is_r11_g11_b10)
          ? PixelBufferDescriptor::PixelDataFormat::RGB
          : PixelBufferDescriptor::PixelDataFormat::RGBA,
      is_r11_g11_b10
          ? PixelBufferDescriptor::PixelDataType::UINT_10F_11F_11F_REV
          : PixelBufferDescriptor::PixelDataType::UBYTE,
      [](void* buffer, size_t size, void* user) {
        auto* texture_upload = reinterpret_cast<TextureUpload*>(user);
        // Sanity checks.
        assert(buffer == texture_upload->image->ImageData());
        assert(size == texture_upload->image->Width() *
                           texture_upload->image->Height() *
                           texture_upload->image->PixelSize());
        if (texture_upload->callback) {
          texture_upload->callback();
        }
        delete texture_upload;
      },
      texture_upload);
}

}  // namespace

absl::StatusOr<std::unique_ptr<ImageContents>> WImageDecodeImage(
    absl::string_view name, resources::Resource resource) {
  absl::string_view encoded_image_view = resource.GetData().StringView();
  WImageBuffer4_b image;

  int width = 0;
  int height = 0;
  int channels = 0;

  if (!WImageIO::DecodeHeader(encoded_image_view, &width, &height, &channels)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Dimensions of image '%s' could not be determined", name));
  }

  if (std::min(width, height) < 1 || std::max(width, height) > kMaxDimension) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Dimensions of image '%s' (%dx%d) exceed limits of (%dx%d)", name,
        width, height, kMaxDimension, kMaxDimension));
  }

  if (!WImageIO::DecodeImage(resource.GetData().StringView(), &image)) {
    return absl::InvalidArgumentError(
        FormatString("Failed to decode image '%.*s' (@%p, %d bytes)",
                     static_cast<int>(name.size()), name.data(),
                     resource.GetData().Data(), resource.GetData().Size()));
  }

  return std::make_unique<WImageImageContents>(std::move(image));
}

}  // namespace imp::image::details
