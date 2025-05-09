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

#ifndef THIRD_PARTY_IMPRESS_CORE_IMAGE_ANDROID_BITMAP_IMAGE_CONTENTS_H_
#define THIRD_PARTY_IMPRESS_CORE_IMAGE_ANDROID_BITMAP_IMAGE_CONTENTS_H_

#include <cstddef>
#include <functional>
#include <memory>

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "core/common/context.h"
#include "core/config.h"

#if IMP_PLATFORM(ANDROID)
#include <android/bitmap.h>
#endif
#include <cstdint>

#include "core/image/image_contents.h"
#include "core/view/platforms/android/wrappers/bitmap.h"

namespace imp::image::details {

class BitmapImageContents : public ImageContents {
 public:
  BitmapImageContents(const imp::Context& context,
                      std::unique_ptr<android::Bitmap> jbitmap,
                      android::BitmapConfig bitmap_config);

  uint32_t GetWidth() const override;
  uint32_t GetStride() const override;
  uint32_t GetHeight() const override;
  size_t GetSize() const override;
  uint8_t* GetData() override;

  bool HasAlpha() const override { return true; }

  filament::backend::PixelBufferDescriptor CreatePixelBufferDescriptor(
      std::function<void()> callback, bool is_r11_g11_b10) override;

  filament::backend::TextureFormat GetTextureFormat() const override;

 private:
  const imp::Context& context_;

  // TODO: Investigate if for some reason the java reference to
  // the bitmap is being leaked, because it seems without calling recycle the
  // bitmap memory never gets cleaned up.
  std::shared_ptr<android::Bitmap> jbitmap_;
  android::BitmapConfig bitmap_config_;

#if IMP_PLATFORM(ANDROID)
  AndroidBitmapInfo bitmap_info_;
#endif
  void* locked_addr_ptr_;

  filament::backend::PixelBufferDescriptor::PixelDataType GetPixelDataType(
      bool is_r11_g11_b10) const;

  filament::backend::PixelBufferDescriptor::PixelDataFormat GetPixelDataFormat(
      bool is_r11_g11_b10) const;
};

}  // namespace imp::image::details

#endif  // THIRD_PARTY_IMPRESS_CORE_IMAGE_ANDROID_BITMAP_IMAGE_CONTENTS_H_
