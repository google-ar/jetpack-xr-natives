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

#ifndef THIRD_PARTY_IMPRESS_CORE_IMAGE_IMAGE_CONTENTS_H_
#define THIRD_PARTY_IMPRESS_CORE_IMAGE_IMAGE_CONTENTS_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"

namespace imp::image {

// Convenience type alias used to abstract and describe the input data passed to
// ImageContents factory functions.
using EncodedImageContents = BufferAccess;

struct CompressedImageContents {
  uint32_t width;
  uint32_t height;
  filament::backend::TextureFormat format;
  std::vector<uint8_t> buffer;
};

class ImageContents {
 public:
  // Stitches six square images into a vertical strip.
  // TODO Refactor this to return a result instead of using an out
  // param.
  static absl::Status CreateStitched(
      std::vector<std::unique_ptr<ImageContents>> images, size_t* offsets,
      std::unique_ptr<ImageContents>* result);

  // Reconstructs an ImageContents given a vector of bytes, a width, and a
  // height.
  //
  // Warning: This function does not validate the layout or format of the image.
  // It is intended to be used to reconstruct an ImageContents, for example,
  // from the result of ImageContents::GetData().
  static absl::StatusOr<std::unique_ptr<ImageContents>> CreatePreStitchedImage(
      int width, int height, std::vector<uint8_t> memory);

  virtual ~ImageContents() {}

  // Return the width of the image.
  virtual uint32_t GetWidth() const = 0;
  // Return the line stride of the image in bytes.
  virtual uint32_t GetStride() const = 0;
  // Return the height of the image.
  virtual uint32_t GetHeight() const = 0;
  // Return the size of the image in bytes.
  virtual std::size_t GetSize() const = 0;
  // Number of mipmap levels. In practice this will be clamped to valid number.
  virtual uint8_t GetLevelCount() const { return 0xff; }
  virtual uint8_t* GetData() = 0;
  // Returns whether or not the image has an alpha channel.
  virtual bool HasAlpha() const = 0;
  // Returns the texture format to match this ImageContents.
  virtual filament::backend::TextureFormat GetTextureFormat() const = 0;
  // Shares image data with a PixelBufferDescriptor and returns it.
  // Only the top level image will be returned.
  virtual filament::backend::PixelBufferDescriptor CreatePixelBufferDescriptor(
      std::function<void()> callback, bool is_r11_g11_b10) = 0;

  // Shares image data and loaded mipmap levels with a
  // vector<PixelBufferDescriptor> and returns it. The mipmap level must be in
  // order starting with the top level. It is valid for mipmaps at higher
  // indices to be missing.
  virtual std::vector<filament::backend::PixelBufferDescriptor>
  CreatePixelBufferDescriptorLevels(std::function<void()> callback,
                                    bool is_r11_g11_b10) {
    std::vector<filament::backend::PixelBufferDescriptor> result;
    result.push_back(CreatePixelBufferDescriptor(callback, is_r11_g11_b10));
    return result;
  }
};

}  // namespace imp::image

#endif  // THIRD_PARTY_IMPRESS_CORE_IMAGE_IMAGE_CONTENTS_H_
