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

#include "core/view/platforms/android/ndkwrappers/image.h"

#include <android/data_space.h>
#include <android/hardware_buffer.h>
#include <dlfcn.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>

#include <memory>

#include "absl/cleanup/cleanup.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "core/common/robin_map.h"
#include "core/config.h"
#include "core/math/mat.h"
#include "core/render/android/android_defines.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::android {

constexpr uint32_t kQCLeftViewMask = 0x01;
constexpr uint32_t kQCRightViewMask = 0x02;

std::unique_ptr<ImageAPIProvider> Image::api_provider = nullptr;

Image::Image(AImage* aimage, AHardwareBuffer* ahardware_buffer)
    : aimage_(aimage),
      ahardware_buffer_(ahardware_buffer),
      is_multiview_(false),
      is_left_primary_(true) {}

absl::StatusOr<std::unique_ptr<Image>> Image::Create(AImage& aimage) {
#if IMP_PLATFORM(ANDROID_API26)
  AHardwareBuffer* ahardware_buffer = nullptr;
  if (AImage_getHardwareBuffer(&aimage, &ahardware_buffer) != AMEDIA_OK) {
    return absl::InternalError("Unable to get hardware buffer from AImage");
  }
  if (ahardware_buffer == nullptr) {
    return absl::InternalError("Image hardware buffer is null.");
  }

  auto image = absl::WrapUnique<Image>(new Image(&aimage, ahardware_buffer));
  absl::Status status = image->UpdateMultivewInfoUsingImageAPIProvider();
  if (!status.ok() && !absl::IsUnavailable(status)) {
    return status;
  }
  return image;
#else
  return absl::UnimplementedError("Requires Android API 26 or higher.");
#endif  // IMP_PLATFORM(ANDROID_API26)
}

absl::Status Image::UpdateMultivewInfoUsingImageAPIProvider() {
  ImageAPIProvider* api_provider = GetImageAPIProvider().get();
  if (!api_provider) {
    IMP_LOG(imp::WARNING) << "ImageAPIProvider is not set. Assuming single view.";
    is_multiview_ = false;
    is_left_primary_ = true;
    auxiliary_view_ahardware_buffer_ = nullptr;
    return absl::UnavailableError("ImageAPIProvider is not set.");
  }

  uint32_t base_view_mask = 0;
  MP_RETURN_IF_ERROR(api_provider->GetBaseView(ahardware_buffer_, base_view_mask));

  uint32_t auxiliary_view_mask = 0;
  MP_RETURN_IF_ERROR(api_provider->GetAuxiliaryViewInfo(ahardware_buffer_,
                                                     auxiliary_view_mask));

  is_multiview_ = false;
  if (base_view_mask != auxiliary_view_mask) {
    MP_RETURN_IF_ERROR(api_provider->GetAuxiliaryBuffer(
        ahardware_buffer_,
        base_view_mask == kQCLeftViewMask ? kQCRightViewMask : kQCLeftViewMask,
        auxiliary_view_ahardware_buffer_));
    is_multiview_ = (auxiliary_view_ahardware_buffer_ != nullptr);
  }
  is_left_primary_ = base_view_mask == kQCLeftViewMask;
  return absl::OkStatus();
}

Image::~Image() {
  // Deleting the native image triggers the return of the buffer to the buffer
  // queue.
#if IMP_PLATFORM(ANDROID_API26)
  if (auxiliary_view_ahardware_buffer_ != nullptr) {
    AHardwareBuffer_release(auxiliary_view_ahardware_buffer_);
  }
  AImage_delete(aimage_);
#endif  // IMP_PLATFORM(ANDROID_API26)
}

bool Image::IsValid() const {
  return aimage_ != nullptr && ahardware_buffer_ != nullptr;
}

absl::StatusOr<int32_t> Image::GetWidth() const {
#if IMP_PLATFORM(ANDROID_API24)
  int32_t width = 0;
  if (AImage_getWidth(aimage_, &width) != AMEDIA_OK) {
    return absl::InternalError("Unable to query the width of AImage");
  }
  return width;
#else   // IMP_PLATFORM(ANDROID_API24)
  return absl::UnimplementedError("Requires Android API 24 or higher.");
#endif  // IMP_PLATFORM(ANDROID_API24)
}

absl::StatusOr<int32_t> Image::GetHeight() const {
#if IMP_PLATFORM(ANDROID_API24)
  int32_t height = 0;
  if (AImage_getHeight(aimage_, &height) != AMEDIA_OK) {
    return absl::InternalError("Unable to query the height of AImage");
  }
  return height;
#else   // IMP_PLATFORM(ANDROID_API24)
  return absl::UnimplementedError("Requires Android API 24 or higher.");
#endif  // IMP_PLATFORM(ANDROID_API24)
}

const AHardwareBuffer* Image::GetHardwareBuffer() const {
  return ahardware_buffer_;
}

absl::StatusOr<ADataSpace> Image::GetBufferDataSpace() const {
  ImageAPIProvider* api_provider = GetImageAPIProvider().get();
  if (api_provider == nullptr) {
    return absl::UnavailableError("ImageAPIProvider is not set.");
  }
  int32_t data_space = ADATASPACE_UNKNOWN;
  MP_RETURN_IF_ERROR(
      api_provider->GetBufferDataSpace(ahardware_buffer_, data_space));
  return static_cast<ADataSpace>(data_space);
}

std::unique_ptr<RobinMap<SurfaceViewType, const AHardwareBuffer*>>
Image::GetViewHardwareBuffers() const {
  std::unique_ptr<RobinMap<SurfaceViewType, const AHardwareBuffer*>>
      view_hardware_buffers = absl::WrapUnique(
          new RobinMap<SurfaceViewType, const AHardwareBuffer*>());
  *view_hardware_buffers = {{SurfaceViewType::kPrimaryView, ahardware_buffer_}};
  if (auxiliary_view_ahardware_buffer_ != nullptr) {
    (*view_hardware_buffers)[SurfaceViewType::kAuxiliaryView] =
        auxiliary_view_ahardware_buffer_;
  }
  return view_hardware_buffers;
}

bool Image::IsMultiview() const { return is_multiview_; }

bool Image::IsLeftPrimary() const { return is_left_primary_; }

absl::StatusOr<mat4f> Image::GetTransformMatrix() const {
  ImageAPIProvider* api_provider = GetImageAPIProvider().get();
  mat4f result;
  if (!api_provider) {
    IMP_LOG(imp::WARNING) << "ImageAPIProvider not available. Returning identity "
                    "transform matrix.";
    return result;
  }

  float matrix[16];
  media_status_t media_status = AMEDIA_ERROR_UNKNOWN;
  absl::Status status =
      api_provider->GetImageTransformMatrix(aimage_, matrix, media_status);

  if (!status.ok()) {
    return absl::InternalError(
        absl::StrCat("Unable to get image transform matrix. Error: ", status));
  }
  if (media_status != AMEDIA_OK) {
    return absl::InternalError(absl::StrCat(
        "Unable to get image transform matrix. media_status Error: ",
        media_status));
  }

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result[i][j] = matrix[i * 4 + j];
    }
  }

  return result;
}

absl::Status Image::SetImageReaderDefaultBufferSize(AImageReader* image_reader,
                                                    uint32_t width,
                                                    uint32_t height) {
  ImageAPIProvider* api_provider = GetImageAPIProvider().get();
  if (!api_provider) {
    IMP_LOG(imp::WARNING)
        << "ImageAPIProvider not available. ImageReader default buffer "
           "size cannot be changed.";
    return absl::OkStatus();
  }

  absl::Status status = api_provider->SetImageReaderDefaultBufferSize(
      image_reader, width, height);
  if (!status.ok()) {
    return absl::InternalError(absl::StrCat(
        "Unable to set ImageReader default buffer size. Error: ", status));
  }
  return absl::OkStatus();
}

void Image::SetImageAPIProvider(std::unique_ptr<ImageAPIProvider> provider) {
  api_provider = std::move(provider);
}

std::unique_ptr<ImageAPIProvider>& Image::GetImageAPIProvider() {
  return api_provider;
}

absl::Status ImageAPIProvider::GetBaseView(const AHardwareBuffer* buffer,
                                           uint32_t& result) {
  return absl::UnimplementedError("GetBaseView is not implemented.");
}

absl::Status ImageAPIProvider::GetAuxiliaryViewInfo(
    const AHardwareBuffer* buffer, uint32_t& result) {
  return absl::UnimplementedError("GetAuxiliaryViewInfo is not implemented.");
}

absl::Status ImageAPIProvider::GetAuxiliaryBuffer(AHardwareBuffer* buffer,
                                                  uint32_t viewMask,
                                                  AHardwareBuffer*& result) {
  return absl::UnimplementedError("GetAuxiliaryBuffer is not implemented.");
}

absl::Status ImageAPIProvider::GetImageTransformMatrix(AImage* image,
                                                       float (&matrix)[16],
                                                       media_status_t& status) {
  return absl::UnimplementedError(
      "GetImageTransformMatrix is not implemented.");
}

absl::Status ImageAPIProvider::GetBufferDataSpace(AHardwareBuffer* buffer,
                                                  int32_t& data_space) {
  return absl::UnimplementedError("GetBufferDataSpace is not implemented.");
}

absl::Status ImageAPIProvider::SetImageReaderDefaultBufferSize(
    AImageReader* image_reader, uint32_t width, uint32_t height) {
  return absl::UnimplementedError(
      "SetImageReaderDefaultBufferSize is not implemented.");
}

}  // namespace imp::android
