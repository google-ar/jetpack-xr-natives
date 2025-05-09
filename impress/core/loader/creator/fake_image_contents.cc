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

#include "core/loader/creator/fake_image_contents.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "core/loader/data/embedded_imp_default_gltf_materials.h"
#include "core/loader/data/embedded_placeholder_textures.h"

namespace imp::loader::details {

filament::backend::PixelBufferDescriptor
FakeImageContents::CreatePixelBufferDescriptor(std::function<void()> callback,
                                               bool is_r11_g11_b10) {
  // Packet whose lifetime begins when a Texture's byte buffer data is queued
  // for consumption by the render thread, and ends when the data is consumed.
  struct TextureUpload {
    void* buffer;
    size_t size;
    std::function<void()> callback;
  };
  TextureUpload* texture_upload =
      new TextureUpload{GetData(), GetSize(), callback};
  return filament::backend::PixelBufferDescriptor(
      GetData(), GetSize(),
      filament::backend::PixelBufferDescriptor::PixelDataFormat::RGBA,
      filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE, 1, 0, 0,
      GetStride() / 4,
      [](void* buffer, size_t size, void* user) {
        auto* texture_upload = reinterpret_cast<TextureUpload*>(user);
        assert(buffer == texture_upload->buffer);
        assert(size == texture_upload->size);
        if (texture_upload->callback) {
          texture_upload->callback();
        }
        delete texture_upload;
      },
      texture_upload);
}

uint8_t FakeImageContents::storage_[4 * 4 * 4];

}  // namespace imp::loader::details
