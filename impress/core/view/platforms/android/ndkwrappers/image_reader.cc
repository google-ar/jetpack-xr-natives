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

#include "core/view/platforms/android/ndkwrappers/image_reader.h"

#include <android/hardware_buffer.h>
#include <android/native_window_jni.h>
#include <dlfcn.h>
#include <jni.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>

#include <cstdint>
#include <memory>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "core/common/context.h"
#include "core/config.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/ndkwrappers/image.h"

namespace imp::android {

ImageReader::ImageReader(const BaseView& view, const int2 size, int32_t format,
                         uint64_t usage, int32_t max_images)
    : view_(view),
      size_(size),
      format_(format),
      usage_(usage),
      max_images_(max_images) {}

absl::Status ImageReader::Initialize() {
#if IMP_PLATFORM(ANDROID_API26)
  media_status_t status = AImageReader_newWithUsage(
      size_.x, size_.y, format_, usage_, max_images_, &reader_);
  if (status != AMEDIA_OK) {
    return absl::InternalError(
        absl::StrCat("Failed to create ImageReader. Status: ", status));
  }

  // Get the Android Surface from the native image reader.
  status = AImageReader_getWindow(reader_, &native_window_);
  if (status != AMEDIA_OK) {
    return absl::InternalError(absl::StrCat(
        "Failed to get window from ImageReader. Status: ", status));
  }

  surface_ =
      ANativeWindow_toSurface(view_.GetContext().GetJniEnv(), native_window_);
  if (surface_ == nullptr) {
    return absl::InternalError("Failed to create surface from native window.");
  }

  return absl::OkStatus();
#else   // IMP_PLATFORM(ANDROID_API26)
  return absl::UnimplementedError("Requires Android API 26 or higher.");
#endif  // IMP_PLATFORM(ANDROID_API26)
}

absl::StatusOr<std::unique_ptr<ImageReader>> ImageReader::Create(
    const BaseView& view, int32_t width, int32_t height, int32_t format,
    uint64_t usage, int32_t max_images) {
  // Create image reader.
  std::unique_ptr<ImageReader> image_reader = absl::WrapUnique<ImageReader>(
      new ImageReader(view, {width, height}, format, usage, max_images));

  MP_RETURN_IF_ERROR(image_reader->Initialize());
  return image_reader;
}

ImageReader::~ImageReader() {
#if IMP_PLATFORM(ANDROID_API26)
  if (reader_) AImageReader_delete(reader_);
#endif  // IMP_PLATFORM(ANDROID_API26)
}

void ImageReader::CallImageListenerCallback() {
  user_callback_(user_callback_context_);
}

absl::Status ImageReader::SetImageListenerCallback(
    void* context, const ImageListenerCallback& callback) {
#if IMP_PLATFORM(ANDROID_API24)
  user_callback_ = callback;
  user_callback_context_ = context;
  listener_.context = this;
  listener_.onImageAvailable = [](void* context, AImageReader* /* reader */) {
    auto reader = static_cast<ImageReader*>(context);
    reader->CallImageListenerCallback();
  };
  if (AImageReader_setImageListener(reader_, &listener_) != AMEDIA_OK) {
    return absl::InternalError("Failed to set image listener.");
  }
  return absl::OkStatus();
#else   // IMP_PLATFORM(ANDROID_API24)
  return absl::UnimplementedError("Requires Android API 24 or higher.");
#endif  // IMP_PLATFORM(ANDROID_API24)
}

absl::Status ImageReader::ResetImageListenerCallback() {
#if IMP_PLATFORM(ANDROID_API24)
  listener_.context = nullptr;
  listener_.onImageAvailable = nullptr;
  if (AImageReader_setImageListener(reader_, &listener_) != AMEDIA_OK) {
    return absl::InternalError("Failed to reset image listener.");
  }
  return absl::OkStatus();
#else   // IMP_PLATFORM(ANDROID_API24)
  return absl::UnimplementedError("Requires Android API 24 or higher.");
#endif  // IMP_PLATFORM(ANDROID_API24)
}

jobject ImageReader::GetSurface() { return surface_; };

absl::StatusOr<std::unique_ptr<Image>> ImageReader::AcquireLatestImage() {
#if IMP_PLATFORM(ANDROID_API24)
  AImage* aimage = nullptr;
  if (AImageReader_acquireLatestImage(reader_, &aimage) != AMEDIA_OK) {
    return absl::InternalError(
        "Failed to acquire lateset image from ImageReader.");
  }
  return Image::Create(*aimage);
#else   // IMP_PLATFORM(ANDROID_API24)
  return absl::UnimplementedError("Requires Android API 24 or higher.");
#endif  // IMP_PLATFORM(ANDROID_API24)
}

absl::Status ImageReader::SetDefaultBufferSize(const int2 size) {
  if (size_ == size) {
    return absl::OkStatus();
  }

  return Image::SetImageReaderDefaultBufferSize(reader_, size.x, size.y);
}

}  // namespace imp::android
