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

#ifndef THIRD_PARTY_IMPRESS_CORE_IMAGE_OWNED_IMAGE_CONTENT_H_
#define THIRD_PARTY_IMPRESS_CORE_IMAGE_OWNED_IMAGE_CONTENT_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "core/image/image_contents.h"

namespace imp::image {
// ImageContents that takes ownership of the data to upload to the texture and
// is responsible for the data's lifetime after that. The data is owned by the
// OwnedImageContents until the CreatePixelBufferDescriptor function is called,
// where ownership of the data is transferred to the PixelBufferDescriptor to
// ensure its lifetime stays until the data is uploaded via Filament APIs (as
// Filament uses the PixelBufferDescriptor to upload the data).
template <class T>
class OwnedImageContents : public image::ImageContents {
 public:
  OwnedImageContents(int width, int height,
                     std::unique_ptr<std::vector<T>> data,
                     filament::backend::TextureFormat format,
                     filament::backend::PixelDataFormat pixel_data_format,
                     filament::backend::PixelDataType pixel_data_type)
      : width_(width),
        height_(height),
        data_(std::move(data)),
        format_(format),
        pixel_data_format_(pixel_data_format),
        pixel_data_type_(pixel_data_type) {}

  uint32_t GetWidth() const override { return width_; }
  uint32_t GetHeight() const override { return height_; }
  uint32_t GetStride() const override { return GetSize() / GetHeight(); }
  std::size_t GetSize() const override { return sizeof(T) * data_->size(); }
  uint8_t GetLevelCount() const override { return 1; }
  uint8_t* GetData() override {
    return reinterpret_cast<uint8_t*>(data_->data());
  }
  bool HasAlpha() const override { return false; }  // Unimplemented
  filament::backend::TextureFormat GetTextureFormat() const override {
    return format_;
  }
  // The returned PixelBufferDescriptor will take ownership of the data and the
  // OwnedImageContent will no longer be valid.
  // We ignore the is_r11_g11_b10 argument since we have member variables that
  // are explicitly for setting up the PixelBufferDescriptor.
  filament::backend::PixelBufferDescriptor CreatePixelBufferDescriptor(
      std::function<void()> callback, bool is_r11_g11_b10) override {
    return filament::backend::PixelBufferDescriptor::make(
        data_->data(), GetSize(), pixel_data_format_, pixel_data_type_,
        [data = std::move(data_), callback = std::move(callback)](
            void* b, size_t s) mutable {
          if (callback) {
            callback();
          }
        });
  }

 private:
  int width_;
  int height_;
  std::unique_ptr<std::vector<T>> data_;
  filament::backend::TextureFormat format_;
  filament::backend::PixelBufferDescriptor::PixelDataFormat pixel_data_format_;
  filament::backend::PixelBufferDescriptor::PixelDataType pixel_data_type_;
};
}  // namespace imp::image

#endif  // THIRD_PARTY_IMPRESS_CORE_IMAGE_OWNED_IMAGE_CONTENT_H_
