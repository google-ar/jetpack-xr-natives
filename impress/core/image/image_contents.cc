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

#include "core/image/image_contents.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"

namespace imp::image {
namespace {
constexpr size_t kFacesPerCube = 6;

using ::filament::backend::PixelBufferDescriptor;
}  // namespace

absl::Status ImageContents::CreateStitched(
    std::vector<std::unique_ptr<ImageContents>> images, size_t* offsets,
    std::unique_ptr<ImageContents>* result) {
  if (images.size() != kFacesPerCube) {
    return absl::InternalError("Wrong number of faces");
  }
  if (!images[0]) {
    return absl::InternalError("Missing first face");
  }
  std::unique_ptr<ImageContents>& first = images.front();
  const std::size_t dim = first->GetWidth();
  const bool has_alpha = first->HasAlpha();
  if (absl::c_any_of(images, [&dim](auto& image) {
        return (image->GetWidth() != dim || image->GetHeight() != dim);
      })) {
    return absl::InternalError("Faces aren't all the same size");
  }

  const std::size_t stitched_image_stride =
      first->GetWidth() * (has_alpha ? 4 : 3);
  const std::size_t face_size = first->GetHeight() * stitched_image_stride;
  const std::size_t stitched_size = face_size * kFacesPerCube;
  std::vector<uint8_t> stitched_image = std::vector<uint8_t>(stitched_size);

  for (size_t face_index = 0; face_index < kFacesPerCube; face_index++) {
    std::unique_ptr<ImageContents>& image = images[face_index];
    offsets[face_index] = face_index * face_size;
    for (uint32_t row = 0; row < image->GetHeight(); ++row) {
      std::copy_n(image->GetData() + row * image->GetStride(),
                  stitched_image_stride,
                  stitched_image.data() + offsets[face_index] +
                      row * stitched_image_stride);
    }
  }
  images.clear();
  *result = std::make_unique<StitchedImageContents>(
      dim, dim * 6, has_alpha ? 4 : 3, std::move(stitched_image));
  assert((*result)->GetSize() == stitched_size);

  return absl::OkStatus();
}

absl::StatusOr<std::unique_ptr<ImageContents>>
ImageContents::CreatePreStitchedImage(int width, int height,
                                      std::vector<uint8_t> memory) {
  if (width <= 0 || height <= 0) {
    return absl::InvalidArgumentError(
        "Both width and height must be positive.");
  }

  // Direct multiplication (width * height) can overflow.
  // So, divide twice.
  const int64_t channels =
      (static_cast<int64_t>(memory.size()) / static_cast<int64_t>(width)) /
      static_cast<int64_t>(height);
  if (channels != 3 && channels != 4) {
    return absl::InvalidArgumentError(
        "The size of the memory buffer is not consistent with the image width,"
        "height, and number of channels.");
  }
  return std::make_unique<StitchedImageContents>(
      width, height, static_cast<int>(channels), std::move(memory));
}

filament::backend::PixelBufferDescriptor
StitchedImageContents::CreatePixelBufferDescriptor(
    std::function<void()> callback, bool is_r11_g11_b10) {
  // Packet whose lifetime begins when a Texture's byte buffer data is queued
  // for consumption by the render thread, and ends when the data is consumed.
  struct TextureUpload {
    std::shared_ptr<std::vector<uint8_t>> memory;
    std::function<void()> callback;
  };
  TextureUpload* texture_upload = new TextureUpload{memory_, callback};

  return PixelBufferDescriptor(
      texture_upload->memory->data(), texture_upload->memory->size(),
      (!HasAlpha() || is_r11_g11_b10)
          ? PixelBufferDescriptor::PixelDataFormat::RGB
          : PixelBufferDescriptor::PixelDataFormat::RGBA,
      is_r11_g11_b10
          ? PixelBufferDescriptor::PixelDataType::UINT_10F_11F_11F_REV
          : PixelBufferDescriptor::PixelDataType::UBYTE,
      [](void* buffer, size_t size, void* user) {
        auto* texture_upload = reinterpret_cast<TextureUpload*>(user);
        // Sanity checks.
        assert(buffer == texture_upload->memory->data());
        assert(size == texture_upload->memory->size());
        if (texture_upload->callback) {
          texture_upload->callback();
        }
        delete texture_upload;
      },
      texture_upload);
}
}  // namespace imp::image
