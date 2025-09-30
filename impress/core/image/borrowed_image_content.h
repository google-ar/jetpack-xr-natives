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

#ifndef THIRD_PARTY_IMPRESS_CORE_IMAGE_BORROWED_IMAGE_CONTENT_H_
#define THIRD_PARTY_IMPRESS_CORE_IMAGE_BORROWED_IMAGE_CONTENT_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "core/image/image_contents.h"

namespace imp::image {
// ImageContents that takes a reference to the data to upload to the texture.
// You are responsible for the data's lifetime.
// This differs from the existing InlineImageContents in that we allow the
// caller to pass in a callback to be executed when the data is uploaded, as
// well as direct specification of the pixel data format and type.
// See also OwnedImageContents, which is a version of this that takes ownership
// of the data.
template <class T>
class BorrowedImageContents : public image::ImageContents {
 public:
  BorrowedImageContents(int width, int height, const std::vector<T>& data,
                        filament::backend::TextureFormat format,
                        filament::backend::PixelDataFormat pixel_data_format,
                        filament::backend::PixelDataType pixel_data_type,
                        std::function<void()> callback = {})
      : width_(width),
        height_(height),
        data_(data),
        format_(format),
        pixel_data_format_(pixel_data_format),
        pixel_data_type_(pixel_data_type),
        callback_(callback) {}

  uint32_t GetWidth() const override { return width_; }
  uint32_t GetHeight() const override { return height_; }
  uint32_t GetStride() const override { return GetSize() / GetHeight(); }
  std::size_t GetSize() const override { return sizeof(T) * data_.size(); }
  uint8_t GetLevelCount() const override { return 1; }
  uint8_t* GetData() override {
    return reinterpret_cast<uint8_t*>(const_cast<T*>(data_.data()));
  }
  bool HasAlpha() const override { return false; }  // Unimplemented
  filament::backend::TextureFormat GetTextureFormat() const override {
    return format_;
  }
  // We ignore the is_r11_g11_b10 argument since we have member variables that
  // are explicitly for setting up the PixelBufferDescriptor.
  filament::backend::PixelBufferDescriptor CreatePixelBufferDescriptor(
      std::function<void()> callback, bool is_r11_g11_b10) override {
    return filament::backend::PixelBufferDescriptor::make(
        data_.data(), GetSize(), pixel_data_format_, pixel_data_type_,
        [image_content_callback = std::move(callback_),
         passed_in_callback = std::move(callback)](void* b, size_t s) mutable {
          if (image_content_callback) {
            image_content_callback();
          }
          if (passed_in_callback) {
            passed_in_callback();
          }
        });
  }

 private:
  int width_;
  int height_;
  const std::vector<T>& data_;
  filament::backend::TextureFormat format_;
  filament::backend::PixelBufferDescriptor::PixelDataFormat pixel_data_format_;
  filament::backend::PixelBufferDescriptor::PixelDataType pixel_data_type_;
  std::function<void()> callback_;
};
}  // namespace imp::image
#endif  // THIRD_PARTY_IMPRESS_CORE_IMAGE_BORROWED_IMAGE_CONTENT_H_
