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

#include "core/image/webp_decode_image.h"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/common/buffer_access.h"
#include "core/image/image_contents.h"
#include "core/resources/resource_manager.h"
#include "third_party/libwebp/src/webp/decode.h"
#include "third_party/libwebp/src/webp/types.h"

namespace imp::image::details {

namespace {

using ::filament::backend::PixelBufferDescriptor;

class WebpImageContents : public ImageContents {
 public:
  WebpImageContents(int width, int stride, int height, int channels,
                    uint8_t* webp_memory)
      : width_(width), stride_(stride), height_(height), channels_(channels) {
    webp_memory_ =
        std::shared_ptr<uint8_t>(webp_memory, [](uint8_t* p) { WebPFree(p); });
  }

  uint32_t GetWidth() const override { return static_cast<uint32_t>(width_); }
  uint32_t GetStride() const override { return static_cast<uint32_t>(stride_); }
  uint32_t GetHeight() const override { return static_cast<uint32_t>(height_); }
  std::size_t GetSize() const override { return width_ * height_ * channels_; }
  uint8_t* GetData() override { return webp_memory_.get(); }
  bool HasAlpha() const override { return channels_ == 4; }
  filament::backend::PixelBufferDescriptor CreatePixelBufferDescriptor(
      std::function<void()> callback, bool is_r11_g11_b10) override;
  filament::backend::TextureFormat GetTextureFormat() const override;

 private:
  int width_;
  int stride_;
  int height_;
  int channels_;
  // Note: We store pixel data as a raw pointer for ease of use with STB API.
  std::shared_ptr<uint8_t> webp_memory_;
};

PixelBufferDescriptor WebpImageContents::CreatePixelBufferDescriptor(
    std::function<void()> callback, bool is_r11_g11_b10) {
  // Packet whose lifetime begins when a Texture's byte buffer data is queued
  // for consumption by the render thread, and ends when the data is consumed.
  struct TextureUpload {
    std::shared_ptr<uint8_t> webp_memory;
    size_t size;
    std::function<void()> callback;
  };
  TextureUpload* texture_upload =
      new TextureUpload{webp_memory_, GetSize(), callback};
  return PixelBufferDescriptor(
      texture_upload->webp_memory.get(), texture_upload->size,
      (!HasAlpha() || is_r11_g11_b10)
          ? PixelBufferDescriptor::PixelDataFormat::RGB
          : PixelBufferDescriptor::PixelDataFormat::RGBA,
      is_r11_g11_b10
          ? PixelBufferDescriptor::PixelDataType::UINT_10F_11F_11F_REV
          : PixelBufferDescriptor::PixelDataType::UBYTE,
      [](void* buffer, size_t size, void* user) {
        auto* texture_upload = reinterpret_cast<TextureUpload*>(user);
        assert(buffer == texture_upload->webp_memory.get());
        assert(size == texture_upload->size);
        if (texture_upload->callback) {
          texture_upload->callback();
        }
        delete texture_upload;
      },
      texture_upload);
}

filament::backend::TextureFormat WebpImageContents::GetTextureFormat() const {
  return HasAlpha() ? filament::Texture::InternalFormat::SRGB8_A8
                    : filament::Texture::InternalFormat::SRGB8;
}
}  // namespace

absl::StatusOr<std::unique_ptr<ImageContents>> WebpDecodeImage(
    absl::string_view name, resources::Resource resource) {
  int desired_channels = 4;
  int width;
  int height;
  uint8_t* image = WebPDecodeRGBA(resource.GetData().Data(),
                                  resource.GetData().Size(), &width, &height);

  if (!image) {
    return absl::InternalError(
        absl::StrFormat("Failed to decode image '%.*s' (@%p, %d bytes)",
                        static_cast<int>(name.size()), name.data(),
                        resource.GetData().Data(), resource.GetData().Size()));
  }

  return std::make_unique<WebpImageContents>(width, width * desired_channels,
                                             height, desired_channels, image);
}

}  // namespace imp::image::details
