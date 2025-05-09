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

#include "core/image/android_bitmap_image_contents.h"

#include <android/bitmap.h>
#include <jni.h>

#include "core/common/log.h"

namespace imp::image::details {

using ::filament::backend::PixelBufferDescriptor;

BitmapImageContents::BitmapImageContents(
    const imp::Context& context, std::unique_ptr<android::Bitmap> jbitmap,
    android::BitmapConfig bitmap_config)
    : context_(context),
      bitmap_config_(bitmap_config),
      bitmap_info_(),
      locked_addr_ptr_(nullptr) {
  jbitmap_ = std::shared_ptr<android::Bitmap>(
      jbitmap.release(), [context](android::Bitmap* bitmap) {
        JNIEnv* env = context.GetJniEnv();
        assert(bitmap->WeakReference());
        AndroidBitmap_unlockPixels(env, bitmap->WeakReference());
        bitmap->Recycle();
      });
  if (!jbitmap_->WeakReference()) {
    IMP_LOG(imp::ERROR) << "Attempt to load null bitmap.";
  }

  JNIEnv* env = context_.GetJniEnv();
  int get_info_result =
      AndroidBitmap_getInfo(env, jbitmap_->WeakReference(), &bitmap_info_);
  if (get_info_result != ANDROID_BITMAP_RESULT_SUCCESS) {
    IMP_LOG(imp::ERROR) << "Failed to read bitmap info.";
    return;
  }

  int lock_pixels_result = AndroidBitmap_lockPixels(
      env, jbitmap_->WeakReference(), &locked_addr_ptr_);
  if (lock_pixels_result != ANDROID_BITMAP_RESULT_SUCCESS) {
    IMP_LOG(imp::ERROR) << "Failed to read bitmap pixels.";
    return;
  }
}

uint32_t BitmapImageContents::GetWidth() const { return bitmap_info_.width; }
uint32_t BitmapImageContents::GetStride() const { return bitmap_info_.stride; }
uint32_t BitmapImageContents::GetHeight() const { return bitmap_info_.height; }
size_t BitmapImageContents::GetSize() const {
  return GetHeight() * GetStride();
}
uint8_t* BitmapImageContents::GetData() {
  return static_cast<uint8_t*>(locked_addr_ptr_);
}

filament::backend::TextureFormat BitmapImageContents::GetTextureFormat() const {
  using filament::backend::TextureFormat;
  switch (bitmap_config_) {
    case android::BitmapConfig::ALPHA_8:
      // TODO: Using RGBA8 is unoptimal, use texture swizzle.
      IMP_LOG(imp::WARNING)
          << "BitmapConfig::ALPHA_8 requested, using TextureFormat::RGBA8";
      return TextureFormat::RGBA8;
    case android::BitmapConfig::ARGB_4444:
      return TextureFormat::RGBA4;
    case android::BitmapConfig::ARGB_8888:
      return TextureFormat::SRGB8_A8;
    case android::BitmapConfig::RGBA_F16:
      return TextureFormat::RGBA16F;
    case android::BitmapConfig::RGB_565:
      return TextureFormat::RGB565;
  }
}

PixelBufferDescriptor BitmapImageContents::CreatePixelBufferDescriptor(
    std::function<void()> callback, bool is_r11_g11_b10) {
  // Packet whose lifetime begins when a Texture's byte buffer data is queued
  // for consumption by the render thread, and ends when the data is consumed.
  struct TextureUpload {
    const imp::Context& context;
    std::shared_ptr<android::Bitmap> jbitmap;
    void* locked_addr_ptr;
    std::size_t size;
    uint32_t stride;
    std::function<void()> callback;
  };
  TextureUpload* texture_upload = new TextureUpload{
      context_, jbitmap_, locked_addr_ptr_, GetSize(), GetStride(), callback};

  return PixelBufferDescriptor(
      texture_upload->locked_addr_ptr, texture_upload->size,
      GetPixelDataFormat(is_r11_g11_b10), GetPixelDataType(is_r11_g11_b10), 1,
      0, 0, GetWidth(),
      [](void* buffer, size_t size, void* user) {
        auto* texture_upload = reinterpret_cast<TextureUpload*>(user);
        assert(buffer == texture_upload->locked_addr_ptr);
        assert(size == texture_upload->size);
        if (texture_upload->callback) {
          texture_upload->callback();
        }
        delete texture_upload;
      },
      texture_upload);
}

PixelBufferDescriptor::PixelDataType BitmapImageContents::GetPixelDataType(
    bool is_r11_g11_b10) const {
  if (is_r11_g11_b10)
    return PixelBufferDescriptor::PixelDataType::UINT_10F_11F_11F_REV;

  switch (bitmap_config_) {
    case android::BitmapConfig::RGBA_F16:
      return PixelBufferDescriptor::PixelDataType::HALF;

    case android::BitmapConfig::ALPHA_8:
    case android::BitmapConfig::ARGB_8888:
      return PixelBufferDescriptor::PixelDataType::UBYTE;

    case android::BitmapConfig::ARGB_4444:
    case android::BitmapConfig::RGB_565:
      return PixelBufferDescriptor::PixelDataType::USHORT;
  }
}

PixelBufferDescriptor::PixelDataFormat BitmapImageContents::GetPixelDataFormat(
    bool is_r11_g11_b10) const {
  if (is_r11_g11_b10) return PixelBufferDescriptor::PixelDataFormat::RGB;

  switch (bitmap_config_) {
    case android::BitmapConfig::RGBA_F16:
    case android::BitmapConfig::ARGB_8888:
    case android::BitmapConfig::ARGB_4444:
      return PixelBufferDescriptor::PixelDataFormat::RGBA;

    case android::BitmapConfig::ALPHA_8:
      return PixelBufferDescriptor::PixelDataFormat::ALPHA;

    case android::BitmapConfig::RGB_565:
      return PixelBufferDescriptor::PixelDataFormat::RGB;
  }
}

}  // namespace imp::image::details
