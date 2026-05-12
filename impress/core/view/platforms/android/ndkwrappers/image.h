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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_NDKWRAPPERS_IMAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_NDKWRAPPERS_IMAGE_H_

#include <android/data_space.h>
#include <android/hardware_buffer.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>

#include <cstdint>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/robin_map.h"
#include "core/math/mat.h"
#include "core/render/android/android_defines.h"

namespace imp::android {

class ImageAPIProvider {
 public:
  virtual absl::Status GetBaseView(const AHardwareBuffer* buffer,
                                   uint32_t& result);

  virtual absl::Status GetAuxiliaryViewInfo(const AHardwareBuffer* buffer,
                                            uint32_t& result);

  virtual absl::Status GetAuxiliaryBuffer(AHardwareBuffer* buffer,
                                          uint32_t viewMask,
                                          AHardwareBuffer*& result);

  virtual absl::Status GetImageTransformMatrix(AImage* image,
                                               float (&matrix)[16],
                                               media_status_t& status);

  virtual absl::Status GetBufferDataSpace(AHardwareBuffer* buffer,
                                          int32_t& data_space);

  virtual absl::Status SetImageReaderDefaultBufferSize(
      AImageReader* image_reader, uint32_t width, uint32_t height);

  virtual ~ImageAPIProvider() = default;
};

// Wrapper for NDK AImage.
class Image {
 public:
  // Image object takes ownership of the AImage. We cannot use unique_ptr
  // because AImage is an opaque type.
  static absl::StatusOr<std::unique_ptr<Image>> Create(AImage& image);

  ~Image();

  Image(const Image& image) = delete;
  Image& operator=(const Image&) = delete;

  // Returns true if the image is valid.
  bool IsValid() const;

  absl::StatusOr<int32_t> GetWidth() const;
  absl::StatusOr<int32_t> GetHeight() const;

  // Returns true if this Image is multiview.
  bool IsMultiview() const;

  // Returns true if this Image is single view or left-primary multiview.
  bool IsLeftPrimary() const;

  // Returns the native hardware buffer of the underlying AImage.
  const AHardwareBuffer* GetHardwareBuffer() const;

  // Returns the dataspace of the buffer of the underlying AImage.
  absl::StatusOr<ADataSpace> GetBufferDataSpace() const;

  // Returns the available <view type, hardware buffer> for the given hardware
  // buffer.
  std::unique_ptr<RobinMap<SurfaceViewType, const AHardwareBuffer*>>
  GetViewHardwareBuffers() const;

  // Returns the transform matrix.
  absl::StatusOr<mat4f> GetTransformMatrix() const;

  // Sets the static API provider for class Image.
  static void SetImageAPIProvider(std::unique_ptr<ImageAPIProvider> provider);

  static absl::Status SetImageReaderDefaultBufferSize(
      AImageReader* image_reader, uint32_t width, uint32_t height);

 private:
  Image(AImage* aimage, AHardwareBuffer* ahardware_buffer);

  AImage* aimage_ = nullptr;
  AHardwareBuffer* ahardware_buffer_ = nullptr;
  AHardwareBuffer* auxiliary_view_ahardware_buffer_ = nullptr;

  bool is_multiview_ = false;
  bool is_left_primary_ = true;

  static std::unique_ptr<ImageAPIProvider> api_provider;

  absl::Status UpdateMultivewInfoUsingImageAPIProvider();

  // Returns the static API provider for class Image.
  static std::unique_ptr<ImageAPIProvider>& GetImageAPIProvider();
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_NDKWRAPPERS_IMAGE_H_
