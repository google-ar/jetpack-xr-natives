/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_FAKE_IMAGE_CONTENTS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_FAKE_IMAGE_CONTENTS_H_

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "core/image/image_contents.h"
#include "core/loader/data/embedded_imp_default_gltf_materials.h"
#include "core/loader/data/embedded_placeholder_textures.h"

namespace imp::loader::details {

class FakeImageContents : public imp::image::ImageContents {
 public:
  uint32_t GetWidth() const override { return 4; }
  uint32_t GetStride() const override { return 4 * GetWidth(); }
  uint32_t GetHeight() const override { return 4; }
  std::size_t GetSize() const override { return GetStride() * GetHeight(); }
  uint8_t* GetData() override { return storage_; }
  bool HasAlpha() const override { return true; }
  filament::backend::TextureFormat GetTextureFormat() const override {
    return filament::backend::TextureFormat::SRGB8_A8;
  }
  filament::backend::PixelBufferDescriptor CreatePixelBufferDescriptor(
      std::function<void()> callback, bool is_r11_g11_b10) override;

 private:
  static uint8_t storage_[4 * 4 * 4];
};

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_FAKE_IMAGE_CONTENTS_H_
