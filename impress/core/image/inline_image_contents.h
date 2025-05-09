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

#ifndef THIRD_PARTY_IMPRESS_CORE_IMAGE_INLINE_IMAGE_CONTENTS_H_
#define THIRD_PARTY_IMPRESS_CORE_IMAGE_INLINE_IMAGE_CONTENTS_H_

#include <stdbool.h>

#include <cstddef>
#include <cstdint>
#include <functional>

#include "absl/log/check.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "core/image/image_contents.h"

namespace imp {
// Allows byte data to be loaded into a texture directly.
//
// data is borrowed during the upload, not owned by this class.
// The caller must ensure data stays valid.
class InlineImageContents : public image::ImageContents {
 public:
  InlineImageContents(int width, int height, const uint8_t* data, uint32_t size,
                      filament::backend::TextureFormat format,
                      bool has_alpha = true)
      : width_(width),
        height_(height),
        stride_(size / height),
        data_(data),
        size_(size),
        format_(format),
        has_alpha_(has_alpha) {
    // verify that the reasonable numbers were passed in.
    
    
  }

  uint32_t GetWidth() const override { return width_; }
  uint32_t GetHeight() const override { return height_; }
  uint32_t GetStride() const override { return stride_; }
  std::size_t GetSize() const override { return GetStride() * GetHeight(); }
  uint8_t GetLevelCount() const override { return 1; }
  uint8_t* GetData() override { return const_cast<uint8_t*>(data_); }
  bool HasAlpha() const override { return has_alpha_; }
  filament::backend::TextureFormat GetTextureFormat() const override {
    return format_;
  }
  filament::backend::PixelBufferDescriptor CreatePixelBufferDescriptor(
      std::function<void()> callback, bool is_r11_g11_b10) override;

 private:
  int width_;
  int height_;
  int stride_;
  const uint8_t* data_;
  uint32_t size_;
  filament::backend::TextureFormat format_;
  bool has_alpha_;
};
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_IMAGE_INLINE_IMAGE_CONTENTS_H_
