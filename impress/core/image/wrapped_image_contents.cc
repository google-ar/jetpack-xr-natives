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

#include "core/image/wrapped_image_contents.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/backend/include/private/backend/BackendUtils.h"

namespace imp::image {

WrappedImageContents::WrappedImageContents(
    uint32_t width, uint32_t height,
    filament::backend::TextureFormat texture_format,
    filament::backend::CompressedPixelDataType compressed_pixel_data_type,
    absl::Span<const uint8_t> data)
    : width_(width),
      height_(height),
      texture_format_(texture_format),
      shared_state_(new SharedState) {
  uint32_t level_width = width_;
  uint32_t level_height = height_;
  std::size_t block_width = filament::backend::getBlockWidth(texture_format);
  std::size_t block_height = filament::backend::getBlockHeight(texture_format);
  std::size_t bytes_per_block =
      filament::backend::getFormatSize(texture_format);
  bool is_compressed = filament::backend::isCompressedFormat(texture_format);
  if (!is_compressed) {
    block_width = 1;
    block_height = 1;
    bytes_per_block = 4;
  }

  // Keep creating a PixelBufferDescriptor while the mip level is larger enough
  // for a compressed block.
  while (level_width >= block_width && level_height >= block_height) {
    std::size_t image_size = (level_width / block_width) *
                             (level_height / block_height) * bytes_per_block;

    // Quits creating new PixelBufferDescriptors if there isn't enough data
    // left.
    if (data.size() < image_size) {
      break;
    }

    constexpr static auto kCallback =
        +[](void* buffer, size_t size, void* user) {
          auto shared_state = reinterpret_cast<SharedState*>(user);
          // Calls the provided callback and deletes the shared state if the
          // last PixelBufferDescriptor has finished.
          if (!--shared_state->pending_upload_count) {
            if (shared_state->callback) {
              shared_state->callback();
            }
            delete shared_state;
          }
        };
    if (is_compressed) {
      pixel_buffer_descriptors_.emplace_back(
          data.data(), image_size, compressed_pixel_data_type, image_size,
          kCallback, shared_state_);
    } else {
      pixel_buffer_descriptors_.emplace_back(
          data.data(), image_size, filament::backend::PixelDataFormat::RGBA,
          filament::backend::PixelDataType::UBYTE, kCallback, shared_state_);
    }

    data.remove_prefix(image_size);
    level_width /= 2;
    level_height /= 2;
  }
  shared_state_->pending_upload_count = pixel_buffer_descriptors_.size();
}

std::vector<filament::backend::PixelBufferDescriptor>
WrappedImageContents::CreatePixelBufferDescriptorLevels(
    std::function<void()> callback, bool is_r11_g11_b10) {
  shared_state_->callback = std::move(callback);
  // TODO (broken link) Fix potentially unsafe invalidation that is happening
  return std::move(pixel_buffer_descriptors_);
}

}  // namespace imp::image
