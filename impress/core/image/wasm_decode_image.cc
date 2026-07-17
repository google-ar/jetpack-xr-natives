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
#include <string>
#include <utility>

#include "zetasql/base/atomic_sequence_num.h"
#include "third_party/GL/gl/include/GLES3/gl3.h"
#include "absl/base/no_destructor.h"
#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/background_delete.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/image/image_contents.h"
#include "core/image/wasm_texture_contents.h"
#include "core/resources/resource_manager.h"

namespace imp::image::details {
namespace {

class WasmImageContents : public ImageContents {
 public:
  explicit WasmImageContents(uint8_t* data, int width, int height)
      : data_(imp::MakeSharedWithBackgroundDeleter<uint8_t[]>(data)),
        width_(width),
        height_(height) {}

  uint32_t GetWidth() const override { return width_; }
  uint32_t GetStride() const override { return width_ * 4; }
  uint32_t GetHeight() const override { return height_; }
  std::size_t GetSize() const override { return width_ * height_ * 4; }
  uint8_t* GetData() override {
    return reinterpret_cast<uint8_t*>(data_.get());
  }
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

class WasmDecodeImageManager {
 public:
  Future<std::unique_ptr<ImageContents>> DecodeImage(
      absl::string_view name, resources::Resource resource);
  Future<WasmTextureContents> DecodeImageToTexture(
      absl::string_view name, resources::Resource resource,
      filament::backend::TextureFormat format, uint8_t requested_levels);

  void OnDecodeImage(uint future_id, intptr_t image, int width, int height);
  void OnDecodeTexture(uint future_id, GLuint texture, int width, int height);
  void OnDecodeError(uint future_id, int decodeTarget, emscripten::val error);

  // TODO: Remove this when removing the
  // enable_label_prep_profile_logging flag.
  void SetLabelPrepProfileLogging(bool enabled) {
    MAIN_THREAD_EM_ASM(
        { Module['wasmDecodeImageManager'].setLabelPrepProfileLogging($0); },
        enabled);
  }

 private:
  // LINT.IfChange
  static const int kDecodeImageToImageData = 0;
  static const int kDecodeImageToTexture = 1;
  // LINT.ThenChange(//depot/google3/third_party/impress/javascript/core/wasm/decode_image/wasm_decode_image.ts)

  uint GetNextFutureID() { return unique_future_id_.GetNext(); }

  GLuint FilamentTextureFormatToGLEnum(
      filament::backend::TextureFormat format) {
    switch (format) {
      case filament::backend::TextureFormat::SRGB8_A8:
        return GL_SRGB8_ALPHA8;
      case filament::backend::TextureFormat::RGBA8:
        return GL_RGBA8;
      case filament::backend::TextureFormat::RGB8:
        return GL_RGB8;
      case filament::backend::TextureFormat::R8:
        return GL_R8;
      case filament::backend::TextureFormat::RG8:
        return GL_RG8;
      case filament::backend::TextureFormat::DEPTH24:
      case filament::backend::TextureFormat::DEPTH32F:
      default:
        IMP_LOG(imp::ERROR) << "Unhandled texture format type, using SRGB8_A8 instead.";
        return GL_SRGB8_ALPHA8;
    }
  }

  absl::flat_hash_map<uint16_t, WeakFuture<std::unique_ptr<ImageContents>>>
      image_futures_;
  absl::flat_hash_map<uint16_t, WeakFuture<WasmTextureContents>>
      texture_futures_;
  base::SequenceNumber unique_future_id_;
};

Future<std::unique_ptr<ImageContents>> WasmDecodeImageManager::DecodeImage(
    absl::string_view name, resources::Resource resource) {
  uint future_id = GetNextFutureID();
  Future<std::unique_ptr<ImageContents>> image_future;
  image_futures_.insert(std::make_pair(future_id, image_future));
  MAIN_THREAD_EM_ASM(
      { Module['wasmDecodeImageManager'].decodeImage($0, $1, $2); }, future_id,
      resource.GetData().Data(), resource.GetData().Size());
  return image_future;
}

Future<WasmTextureContents> WasmDecodeImageManager::DecodeImageToTexture(
    absl::string_view name, resources::Resource resource,
    filament::backend::TextureFormat format, uint8_t requested_levels) {
  uint future_id = GetNextFutureID();
  Future<WasmTextureContents> texture_future;
  texture_futures_.insert(std::make_pair(future_id, texture_future));
  GLuint texture;
  glGenTextures(1, &texture);
  GLuint texture_format = FilamentTextureFormatToGLEnum(format);
  MAIN_THREAD_EM_ASM(
      {
        Module['wasmDecodeImageManager'].decodeImageToTexture(
            $0, $1, $2, GL.textures[$3], $3, $4, $5);
      },
      future_id, resource.GetData().Data(), resource.GetData().Size(), texture,
      texture_format, requested_levels);
  return texture_future;
}

void WasmDecodeImageManager::OnDecodeImage(uint future_id, intptr_t image,
                                           int width, int height) {
  if (!image_futures_.contains(future_id)) {
    IMP_LOG(imp::ERROR) << "Future ID not found when resolving onDecodeImage";
    return;
  }
  absl::optional<Future<std::unique_ptr<ImageContents>>> image_future =
      image_futures_.at(future_id).Lock();
  image_futures_.erase(future_id);

  if (image_future.has_value()) {
    image_future->Return(std::make_unique<WasmImageContents>(
        reinterpret_cast<uint8_t*>(image), width, height));
  }
}
void WasmDecodeImageManager::OnDecodeTexture(uint future_id, GLuint texture,
                                             int width, int height) {
  if (!texture_futures_.contains(future_id)) {
    IMP_LOG(imp::ERROR) << "Future ID not found when resolving onDecodeTexture";
    return;
  }
  absl::optional<Future<WasmTextureContents>> texture_future =
      texture_futures_.at(future_id).Lock();
  texture_futures_.erase(future_id);
  if (texture_future.has_value()) {
    texture_future->Return(WasmTextureContents(texture, width, height));
  }
}
void WasmDecodeImageManager::OnDecodeError(uint future_id, int decodeTarget,
                                           emscripten::val error) {
  if (!image_futures_.contains(future_id) &&
      !texture_futures_.contains(future_id)) {
    IMP_LOG(imp::ERROR) << "Future ID not found when resolving "
               << (decodeTarget == kDecodeImageToImageData ? "decodeImage"
                                                           : "decodeTexture")
               << " with error: " << error.as<std::string>();
    return;
  }
  switch (decodeTarget) {
    case kDecodeImageToImageData: {
      absl::optional<Future<std::unique_ptr<ImageContents>>> image_future =
          image_futures_.at(future_id).Lock();
      image_futures_.erase(future_id);
      if (image_future.has_value()) {
        image_future->Return(absl::InternalError(error.as<std::string>()));
      }
      break;
    }
    case kDecodeImageToTexture: {
      absl::optional<Future<WasmTextureContents>> texture_future =
          texture_futures_.at(future_id).Lock();
      texture_futures_.erase(future_id);
      if (texture_future.has_value()) {
        texture_future->Return(absl::InternalError(error.as<std::string>()));
      }
      break;
    }
    default:
      IMP_LOG(imp::ERROR) << "Unknown decode target: " << decodeTarget
                 << " with error: " << error.as<std::string>();
      break;
  }
}

WasmDecodeImageManager* GetDecodeImageManager() {
  static absl::NoDestructor<WasmDecodeImageManager> decode_image_manager;
  return decode_image_manager.get();
}

void OnDecodeImage(uint future_id, intptr_t image, int width, int height) {
  return GetDecodeImageManager()->OnDecodeImage(future_id, image, width,
                                                height);
}

void OnDecodeTexture(uint future_id, GLuint texture, int width, int height) {
  return GetDecodeImageManager()->OnDecodeTexture(future_id, texture, width,
                                                  height);
}

void OnDecodeError(uint future_id, int decodeTarget, emscripten::val error) {
  return GetDecodeImageManager()->OnDecodeError(future_id, decodeTarget, error);
}

EMSCRIPTEN_BINDINGS(decode_image_bindings) {
  emscripten::function("onDecodeImage", imp::image::details::OnDecodeImage,
                       emscripten::allow_raw_pointers());
  emscripten::function("onDecodeTexture", imp::image::details::OnDecodeTexture);
  emscripten::function("onDecodeError", imp::image::details::OnDecodeError);
};
}  // namespace

void SetLabelPrepProfileLogging(bool enabled) {
  return GetDecodeImageManager()->SetLabelPrepProfileLogging(enabled);
}

Future<std::unique_ptr<ImageContents>> WasmDecodeImage(
    absl::string_view name, resources::Resource resource) {
  return GetDecodeImageManager()->DecodeImage(name, resource);
}

Future<WasmTextureContents> WasmDecodeImageToTexture(
    absl::string_view name, resources::Resource resource,
    filament::backend::TextureFormat format, uint8_t requested_levels) {
  if (!Executor::IsOnForegroundExecutor()) {
    IMP_LOG(imp::FATAL)
        << "WasmDecodeImageToTexture must be called on the foreground thread.";
  }
  return GetDecodeImageManager()->DecodeImageToTexture(name, resource, format,
                                                       requested_levels);
}

}  // namespace imp::image::details
