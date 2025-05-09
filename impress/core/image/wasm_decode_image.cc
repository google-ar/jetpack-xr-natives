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

#include "core/image/wasm_decode_image.h"

#include <emscripten/bind.h>
#include <emscripten/emscripten.h>
#include <emscripten/val.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <utility>

#include "zetasql/base/atomic_sequence_num.h"
#include "absl/base/no_destructor.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/robin_map.h"
#include "core/image/image_contents.h"
#include "core/resources/resource_manager.h"

namespace imp::image::details {
namespace {

class WasmImageContents : public ImageContents {
 public:
  explicit WasmImageContents(uint8_t* data, int width, int height)
      : data_(data), width_(width), height_(height) {}

  uint32_t GetWidth() const override { return width_; }
  uint32_t GetStride() const override { return width_ * 4; }
  uint32_t GetHeight() const override { return height_; }
  std::size_t GetSize() const override { return width_ * height_ * 4; }
  uint8_t* GetData() override { return data_.get(); }
  bool HasAlpha() const override { return true; }
  filament::backend::TextureFormat GetTextureFormat() const override {
    return filament::Texture::InternalFormat::SRGB8_A8;
  }
  filament::backend::PixelBufferDescriptor CreatePixelBufferDescriptor(
      std::function<void()> callback, bool is_r11_g11_b10) {
    // Packet whose lifetime begins when a Texture's byte buffer data is queued
    // for consumption by the render thread, and ends when the data is consumed.
    struct TextureUpload {
      std::shared_ptr<uint8_t> data;
      size_t size;
      std::function<void()> callback;
    };
    TextureUpload* texture_upload =
        new TextureUpload{data_, GetSize(), callback};
    return filament::backend::PixelBufferDescriptor(
        texture_upload->data.get(), texture_upload->size,
        is_r11_g11_b10
            ? filament::backend::PixelBufferDescriptor::PixelDataFormat::RGB
            : filament::backend::PixelBufferDescriptor::PixelDataFormat::RGBA,
        is_r11_g11_b10
            ? filament::backend::PixelBufferDescriptor::PixelDataType::
                  UINT_10F_11F_11F_REV
            : filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE,
        [](void* buffer, size_t size, void* user) {
          auto* texture_upload = reinterpret_cast<TextureUpload*>(user);
          // Sanity checks.
          
          assert(size == texture_upload->size);
          if (texture_upload->callback) {
            texture_upload->callback();
          }
          delete texture_upload;
        },
        texture_upload);
  }

 private:
  std::shared_ptr<uint8_t> data_;
  uint32_t width_;
  uint32_t height_;
};

using IdsToImageFutures =
    RobinMap<uint16_t, WeakFuture<std::unique_ptr<ImageContents>>>;

IdsToImageFutures& GetIdsToImageFutures() {
  static absl::NoDestructor<IdsToImageFutures> ids_to_image_futures;
  return *ids_to_image_futures;
}

EMSCRIPTEN_BINDINGS(decode_image_bindings) {
  emscripten::function(
      "onDecodeImage",
      +[](uint future_id, intptr_t image, int width, int height,
          emscripten::val error) {
        if (!GetIdsToImageFutures().contains(future_id)) {
          IMP_LOG(imp::ERROR) << "Future ID not found when resolving onDecodeImage";
          return;
        }
        absl::optional<Future<std::unique_ptr<ImageContents>>> image_future =
            GetIdsToImageFutures().at(future_id).Lock();
        GetIdsToImageFutures().erase(future_id);

        if (image_future.has_value()) {
          if (image == -1) {
            image_future->Return(absl::InternalError(error.as<std::string>()));
          } else {
            image_future->Return(std::make_unique<WasmImageContents>(
                reinterpret_cast<uint8_t*>(image), width, height));
          }
        }
      },
      emscripten::allow_raw_pointers());
};

uint GetNextFutureID() {
  static base::SequenceNumber unique_future_id;
  return unique_future_id.GetNext();
}
}  // namespace

Future<std::unique_ptr<ImageContents>> WasmDecodeImage(
    absl::string_view name, resources::Resource resource) {
  uint future_id = GetNextFutureID();
  Future<std::unique_ptr<ImageContents>> image_future;
  GetIdsToImageFutures().insert(std::make_pair(future_id, image_future));
  MAIN_THREAD_EM_ASM(
      { Module['wasmDecodeImageManager'].decodeImage($0, $1, $2); }, future_id,
      resource.GetData().Data(), resource.GetData().Size());
  return image_future;
}

}  // namespace imp::image::details
