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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_NDKWRAPPERS_IMAGE_READER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_NDKWRAPPERS_IMAGE_READER_H_

#include <android/native_window_jni.h>
#include <jni.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>

#include <cstdint>
#include <functional>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/context.h"
#include "core/config.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/ndkwrappers/image.h"

namespace imp::android {

// Wrapper for NDK AImageReader.
class ImageReader {
 public:
  static absl::StatusOr<std::unique_ptr<ImageReader>> Create(
      const BaseView& view, int32_t width, int32_t height, int32_t format,
      uint64_t usage, int32_t max_images);

  ~ImageReader();

  // Return the surface from the native image reader.
  jobject GetSurface();

  // Acquire the most recent Image from the underlying AImageReader.
  absl::StatusOr<std::unique_ptr<Image>> AcquireLatestImage();

  // Register a callback function to be executed when a new image becomes
  // available. The context parameter can be used in the callback to access the
  // object that owns the ImageReader.
  using ImageListenerCallback = std::function<void(void* context)>;
  absl::Status SetImageListenerCallback(void* context,
                                        const ImageListenerCallback& callback);

  // Disable any existing user-defined image callback.
  absl::Status ResetImageListenerCallback();

  // Set the buffer size of the ImageReader.
  absl::Status SetDefaultBufferSize(int2 size);

 private:
  // Create ImageReader with the given size, format and usage.
  ImageReader(const BaseView& view, int2 size, int32_t format, uint64_t usage,
              int32_t max_images);

  absl::Status Initialize();

  void CallImageListenerCallback();

  const BaseView& view_;
  int2 size_ = {0, 0};
  const int32_t format_ = 0;
  const uint64_t usage_ = 0;
  const int32_t max_images_ = 0;

  AImageReader* reader_ = nullptr;
  AImageReader_ImageListener listener_{nullptr, nullptr};
  ANativeWindow* native_window_ = nullptr;

  jobject surface_;

  void* user_callback_context_;
  ImageListenerCallback user_callback_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_NDKWRAPPERS_IMAGE_READER_H_
