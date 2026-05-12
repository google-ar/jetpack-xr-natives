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

#include "core/image/stb_decode_image.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/executor.h"
#include "core/common/buffer_access.h"
#include "core/image/image_contents.h"
#include "core/resources/resource_manager.h"
#include "stblib/stb_image.h"

namespace imp::image::details {
namespace {

using ::filament::backend::PixelBufferDescriptor;

void ReleaseStorage(uint8_t* stb_memory) {
  // According to the documentation this just calls free().
  stbi_image_free(stb_memory);
}

bool InitializeStb() {
  // PNGs that are packaged via xcode for iPhones undergo unwanted
  // premultiplication. Fortunately, Apple put a bit in the png header to
  // denote that they've done this so that libraries like stbi can detect and
  // undo the premultiplication. Enabling these 2 stbi options (as per their
  // documentation in google3/third_party/stblib/stb_image.h) normalizes
  // iPhone pngs.
  stbi_convert_iphone_png_to_rgb(1);
  stbi_set_unpremultiply_on_load(1);
  return true;
}
static const bool kStbInitialized = InitializeStb();

class StbImageContents : public ImageContents {
 public:
  StbImageContents(int width, int stride, int height, int channels,
                   uint8_t* stb_memory)
      : width_(width), stride_(stride), height_(height), channels_(channels) {
    stb_memory_ = std::shared_ptr<uint8_t>(stb_memory, [](uint8_t* p) {
      if (Executor::BackgroundExecutor() != nullptr) {
        Executor::BackgroundExecutor()->ScheduleInvocable(
            [p]() { ReleaseStorage(p); });
      } else {
        ReleaseStorage(p);
      }
    });
  }

  uint32_t GetWidth() const override { return static_cast<uint32_t>(width_); }
  uint32_t GetStride() const override { return static_cast<uint32_t>(stride_); }
  uint32_t GetHeight() const override { return static_cast<uint32_t>(height_); }
  std::size_t GetSize() const override { return width_ * height_ * channels_; }
  uint8_t* GetData() override { return stb_memory_.get(); }
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
  std::shared_ptr<uint8_t> stb_memory_;
};

PixelBufferDescriptor StbImageContents::CreatePixelBufferDescriptor(
    std::function<void()> callback, bool is_r11_g11_b10) {
  // Packet whose lifetime begins when a Texture's byte buffer data is queued
  // for consumption by the render thread, and ends when the data is consumed.
  struct TextureUpload {
    std::shared_ptr<uint8_t> stb_memory;
    size_t size;
    std::function<void()> callback;
  };
  TextureUpload* texture_upload =
      new TextureUpload{stb_memory_, GetSize(), callback};
  return PixelBufferDescriptor(
      texture_upload->stb_memory.get(), texture_upload->size,
      (!HasAlpha() || is_r11_g11_b10)
          ? PixelBufferDescriptor::PixelDataFormat::RGB
          : PixelBufferDescriptor::PixelDataFormat::RGBA,
      is_r11_g11_b10
          ? PixelBufferDescriptor::PixelDataType::UINT_10F_11F_11F_REV
          : PixelBufferDescriptor::PixelDataType::UBYTE,
      [](void* buffer, size_t size, void* user) {
        auto* texture_upload = reinterpret_cast<TextureUpload*>(user);
        // Sanity checks.
        assert(buffer == texture_upload->stb_memory.get());
        assert(size == texture_upload->size);
        if (texture_upload->callback) {
          texture_upload->callback();
        }
        delete texture_upload;
      },
      texture_upload);
}

filament::backend::TextureFormat StbImageContents::GetTextureFormat() const {
  return HasAlpha() ? filament::Texture::InternalFormat::SRGB8_A8
                    : filament::Texture::InternalFormat::SRGB8;
}

const char base64_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64_encode(const uint8_t* data, size_t len) {
  std::string encoded_string;
  int i = 0;
  int j = 0;
  uint8_t char_array_3[3];
  uint8_t char_array_4[4];

  for (size_t k = 0; k < len; k++) {
    char_array_3[i++] = data[k];
    if (i == 3) {
      char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
      char_array_4[1] =
          ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
      char_array_4[2] =
          ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
      char_array_4[3] = char_array_3[2] & 0x3f;

      for (i = 0; (i < 4); i++) encoded_string += base64_chars[char_array_4[i]];
      i = 0;
    }
  }

  if (i) {
    for (j = i; j < 3; j++) char_array_3[j] = '\0';

    char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
    char_array_4[1] =
        ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
    char_array_4[2] =
        ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
    char_array_4[3] = char_array_3[2] & 0x3f;

    for (j = 0; (j < i + 1); j++)
      encoded_string += base64_chars[char_array_4[j]];

    while ((i++ < 3)) encoded_string += '=';
  }

  return encoded_string;
}
}  // namespace

absl::StatusOr<std::unique_ptr<ImageContents>> StbDecodeImage(
    absl::string_view name, resources::Resource resource,
    bool fatal_on_pink_texture) {
  
  int desired_channels = 4;
  int width;
  int height;
  int channels;
  uint8_t* image = stbi_load_from_memory(
      reinterpret_cast<stbi_uc const*>(resource.GetData().Data()),
      static_cast<int>(resource.GetData().Size()), &width, &height, &channels,
      desired_channels);
  if (!image) {
    return absl::InternalError(
        absl::StrFormat("Failed to decode image '%.*s' (@%p, %d bytes)",
                        static_cast<int>(name.size()), name.data(),
                        resource.GetData().Data(), resource.GetData().Size()));
  }

  // (broken link) Trying to detect pink decoding issue by checking if all pixels
  // are pink.
  bool all_pixels_pink = true;
  for (int i = 0; i < width * height * desired_channels;
       i += desired_channels) {
    if (image[i] != 255 || image[i + 1] != 0 || image[i + 2] != 255 ||
        image[i + 3] != 255) {
      all_pixels_pink = false;
      break;
    }
  }
  if (all_pixels_pink) {
    IMP_LOG(imp::ERROR) << "Pink decoding issue detected. All pixels were pink.";
    if (fatal_on_pink_texture) {
      IMP_LOG(imp::ERROR) << name;
      IMP_LOG(imp::ERROR) << "data:image/png;base64,"
                 << base64_encode(resource.GetData().Data(),
                                  resource.GetData().Size());
      IMP_LOG(imp::FATAL) << "IF YOU SEE THIS PLEASE CONTACT FAALAND@ WITH THE TWO "
                    "PREVIOUS LOG STATEMENTS";
    }
  }

  return std::make_unique<StbImageContents>(width, width * desired_channels,
                                            height, desired_channels, image);
}

}  // namespace imp::image::details
