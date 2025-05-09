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

#ifndef THIRD_PARTY_IMPRESS_CORE_IMAGE_WRAPPED_IMAGE_CONTENTS_H_
#define THIRD_PARTY_IMPRESS_CORE_IMAGE_WRAPPED_IMAGE_CONTENTS_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "core/image/image_contents.h"

namespace imp::image {

// Implementation of ImageContents built from a view of existing pixel data.
// This implements only the parts of the ImageContents interface which are
// useful for glTF texturing.
class WrappedImageContents : public ImageContents {
 public:
  WrappedImageContents(
      uint32_t width, uint32_t height,
      filament::backend::TextureFormat texture_format,
      filament::backend::CompressedPixelDataType compressed_pixel_data_type,
      absl::Span<const uint8_t> data);

  uint32_t GetWidth() const override { return width_; }
  uint32_t GetStride() const override { return 0; }  // Unimplemented
  uint32_t GetHeight() const override { return height_; }
  std::size_t GetSize() const override { return 0; }  // Unimplemented
  uint8_t* GetData() override { return nullptr; }     // Unimplemented
  bool HasAlpha() const override { return false; }    // Unimplemented
  filament::backend::TextureFormat GetTextureFormat() const override {
    return texture_format_;
  }

  // Unimplemented
  filament::backend::PixelBufferDescriptor CreatePixelBufferDescriptor(
      std::function<void()> callback, bool is_r11_g11_b10) override {
    return {};
  }

  std::vector<filament::backend::PixelBufferDescriptor>
  CreatePixelBufferDescriptorLevels(std::function<void()> callback,
                                    bool is_r11_g11_b10) override;

  uint8_t GetLevelCount() const override {
    return pixel_buffer_descriptors_.size();
  }

 private:
  // Tracks the PixelBufferDescriptors being uploaded by filament,
  struct SharedState {
    std::function<void()> callback;
    uint32_t pending_upload_count;
  };

  uint32_t width_;
  uint32_t height_;
  filament::backend::TextureFormat texture_format_;
  std::vector<filament::backend::PixelBufferDescriptor>
      pixel_buffer_descriptors_;
  SharedState* shared_state_;
};

}  // namespace imp::image

#endif  // THIRD_PARTY_IMPRESS_CORE_IMAGE_WRAPPED_IMAGE_CONTENTS_H_
