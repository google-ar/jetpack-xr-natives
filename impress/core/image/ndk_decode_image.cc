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

#include "core/image/ndk_decode_image.h"

#include <android/bitmap.h>
#include <android/imagedecoder.h>

#include <memory>
#include <string>

#include "absl/memory/memory.h"
#include "absl/status/statusor.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/common/platform_helpers.h"
#include "core/image/image_contents.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::image::details {
namespace {

using ::filament::backend::PixelBufferDescriptor;

class NdkImageContents : public ImageContents {
 public:
  NdkImageContents(int width, int stride, int height,
                   std::unique_ptr<uint8_t> pixels)
      : width_(width),
        stride_(stride),
        height_(height),
        pixels_(std::move(pixels)) {}

  ~NdkImageContents() override {}

  uint32_t GetWidth() const override { return static_cast<uint32_t>(width_); }
  uint32_t GetStride() const override { return static_cast<uint32_t>(stride_); }
  uint32_t GetHeight() const override { return static_cast<uint32_t>(height_); }
  std::size_t GetSize() const override { return height_ * stride_; }
  uint8_t* GetData() override { return pixels_.get(); }
  bool HasAlpha() const override { return true; }
  filament::backend::PixelBufferDescriptor CreatePixelBufferDescriptor(
      std::function<void()> callback, bool is_r11_g11_b10) override;
  filament::backend::TextureFormat GetTextureFormat() const override;

 private:
  int width_;
  int stride_;
  int height_;
  std::shared_ptr<uint8_t> pixels_;
};

PixelBufferDescriptor NdkImageContents::CreatePixelBufferDescriptor(
    std::function<void()> callback, bool is_r11_g11_b10) {
  // Packet whose lifetime begins when a Texture's byte buffer data is queued
  // for consumption by the render thread, and ends when the data is consumed.
  struct TextureUpload {
    std::shared_ptr<uint8_t> pixels;
    size_t size;
    std::function<void()> callback;
  };
  TextureUpload* texture_upload =
      new TextureUpload{pixels_, GetSize(), callback};
  return PixelBufferDescriptor(
      texture_upload->pixels.get(), texture_upload->size,
      (!HasAlpha() || is_r11_g11_b10)
          ? PixelBufferDescriptor::PixelDataFormat::RGB
          : PixelBufferDescriptor::PixelDataFormat::RGBA,
      is_r11_g11_b10
          ? PixelBufferDescriptor::PixelDataType::UINT_10F_11F_11F_REV
          : PixelBufferDescriptor::PixelDataType::UBYTE,
      [](void* buffer, size_t size, void* user) {
        auto* texture_upload = reinterpret_cast<TextureUpload*>(user);
        assert(buffer == texture_upload->pixels.get());
        assert(size == texture_upload->size);
        if (texture_upload->callback) {
          texture_upload->callback();
        }
        delete texture_upload;
      },
      texture_upload);
}

filament::backend::TextureFormat NdkImageContents::GetTextureFormat() const {
  return filament::Texture::InternalFormat::SRGB8_A8;
}
}  // namespace

absl::StatusOr<std::unique_ptr<ImageContents>> NdkDecodeImage(
    resources::Resource resource) {
  if (__builtin_available(android 30, *)) {
    AImageDecoder* decoder;
    int result = AImageDecoder_createFromBuffer(
        resource.GetData().Data(), resource.GetData().Size(), &decoder);
    if (result != ANDROID_IMAGE_DECODER_SUCCESS) {
      return absl::InternalError(
          "The file can not be decoded by ndk image decoder");
    }
    AImageDecoder_setAndroidBitmapFormat(decoder,
                                         ANDROID_BITMAP_FORMAT_RGBA_8888);

    AImageDecoder_setUnpremultipliedRequired(decoder, true);
    const AImageDecoderHeaderInfo* info = AImageDecoder_getHeaderInfo(decoder);
    int32_t width = AImageDecoderHeaderInfo_getWidth(info);
    int32_t height = AImageDecoderHeaderInfo_getHeight(info);
    size_t stride = AImageDecoder_getMinimumStride(decoder);
    size_t size = height * stride;

    std::unique_ptr<uint8_t> pixels(new uint8_t[size]);

    result = AImageDecoder_decodeImage(decoder, pixels.get(), stride, size);
    // We’re done with the decoder, so now it’s safe to delete it.
    AImageDecoder_delete(decoder);

    if (result != ANDROID_IMAGE_DECODER_SUCCESS) {
      return absl::InternalError("Failed to decode image");
    }

    return std::make_unique<NdkImageContents>(width, stride, height,
                                              std::move(pixels));
  }
  return absl::InternalError("Not supported");
}
}  // namespace imp::image::details
