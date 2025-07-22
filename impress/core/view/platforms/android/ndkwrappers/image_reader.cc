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
#include "core/view/base_view.h"
#include "core/view/platforms/android/ndkwrappers/image.h"

namespace imp::android {

ImageReader::ImageReader(int32_t width, int32_t height, int32_t format,
                         uint64_t usage, int32_t max_images) {
  reader_ = nullptr;
  AImageReader_newWithUsage(width, height, format, usage, max_images, &reader_);
}

absl::StatusOr<std::unique_ptr<ImageReader>> ImageReader::Create(
    const BaseView& view, int32_t width, int32_t height, int32_t format,
    uint64_t usage, int32_t max_images) {
  // Create image reader.
  std::unique_ptr<ImageReader> image_reader = absl::WrapUnique<ImageReader>(
      new ImageReader(width, height, format, usage, max_images));
  if (image_reader->reader_ == nullptr) {
    return absl::InternalError("ImageReader is not initialized.");
  }

  // Get the Android Surface from the native image reader.
  ANativeWindow* native_window;
  if (AImageReader_getWindow(image_reader->reader_, &native_window) !=
      AMEDIA_OK) {
    return absl::InternalError("Failed to get window from ImageReader.");
  }
  image_reader->surface_ =
      ANativeWindow_toSurface(view.GetContext().GetJniEnv(), native_window);

  return image_reader;
}

ImageReader::~ImageReader() {
  if (reader_) AImageReader_delete(reader_);
}

void ImageReader::CallImageListenerCallback() {
  user_callback_(user_callback_context_);
}

absl::Status ImageReader::SetImageListenerCallback(
    void* context, const ImageListenerCallback& callback) {
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
}

absl::Status ImageReader::ResetImageListenerCallback() {
  listener_.context = nullptr;
  listener_.onImageAvailable = nullptr;
  if (AImageReader_setImageListener(reader_, &listener_) != AMEDIA_OK) {
    return absl::InternalError("Failed to reset image listener.");
  }
  return absl::OkStatus();
}

jobject ImageReader::GetSurface() { return surface_; };

absl::StatusOr<std::unique_ptr<Image>> ImageReader::AcquireLatestImage() {
  AImage* aimage = nullptr;
  if (AImageReader_acquireLatestImage(reader_, &aimage) != AMEDIA_OK) {
    return absl::InternalError(
        "Failed to acquire lateset image from ImageReader.");
  }
  return Image::Create(*aimage);
}

}  // namespace imp::android
