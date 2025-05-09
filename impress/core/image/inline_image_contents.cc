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

#include "core/image/inline_image_contents.h"

#include <cstddef>
#include <cstdint>
#include <functional>

#include "absl/log/check.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"

namespace imp {
using ::filament::backend::PixelBufferDescriptor;

filament::backend::PixelBufferDescriptor
InlineImageContents::CreatePixelBufferDescriptor(std::function<void()> callback,
                                                 bool is_r11_g11_b10) {
  // Packet whose lifetime begins when a Texture's byte buffer data is
  // queued for consumption by the render thread, and ends when the data is
  // consumed.
  struct TextureUpload {
    // data is borrowed during the upload not owned.
    const uint8_t* data;
    size_t size;
    filament::backend::PixelDataFormat pixel_format;
    std::function<void()> callback;
  };
  PixelBufferDescriptor::PixelDataFormat pixel_format =
      (!has_alpha_ || is_r11_g11_b10)
          ? PixelBufferDescriptor::PixelDataFormat::RGB
          : PixelBufferDescriptor::PixelDataFormat::RGBA;

  TextureUpload* texture_upload =
      new TextureUpload{data_, size_, pixel_format, callback};

  return PixelBufferDescriptor(
      texture_upload->data, texture_upload->size, texture_upload->pixel_format,
      PixelBufferDescriptor::PixelDataType::UBYTE,
      [](void* buffer, size_t size, void* user) {
        auto* texture_upload = reinterpret_cast<TextureUpload*>(user);

        
        
        if (texture_upload->callback) {
          texture_upload->callback();
        }
        delete texture_upload;
      },
      texture_upload);
}
}  // namespace imp
