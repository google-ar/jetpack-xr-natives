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
#include "core/math/mat.h"
#include "core/render/android/android_defines.h"
#include "core/view/platforms/android/ndkwrappers/hardware_buffer_helper.h"

namespace imp::android {

constexpr uint32_t kQCAuxiliaryViewMask = 0x02;

std::unique_ptr<ImageAPIProvider> Image::api_provider = nullptr;

Image::Image(AImage* aimage, AHardwareBuffer* ahardware_buffer)
    : aimage_(aimage), ahardware_buffer_(ahardware_buffer) {}

absl::StatusOr<std::unique_ptr<Image>> Image::Create(AImage& aimage) {
  AHardwareBuffer* ahardware_buffer = nullptr;
  if (AImage_getHardwareBuffer(&aimage, &ahardware_buffer) != AMEDIA_OK) {
    return absl::InternalError("Unable to get hardware buffer from AImage");
  }

  return absl::WrapUnique<Image>(new Image(&aimage, ahardware_buffer));
}

Image::~Image() {
  // Deleting the native image triggers the return of the buffer to the buffer
  // queue.
  if (auxiliary_view_buffer_holder_ != nullptr) {
    AHardwareBuffer_release(auxiliary_view_buffer_holder_);
  }
  AImage_delete(aimage_);
}

bool Image::IsValid() const {
  return aimage_ != nullptr && ahardware_buffer_ != nullptr;
}

absl::StatusOr<int32_t> Image::GetWidth() const {
  int32_t width = 0;
  if (AImage_getWidth(aimage_, &width) != AMEDIA_OK) {
    return absl::InternalError("Unable to query the width of AImage");
  }
  return width;
}

absl::StatusOr<int32_t> Image::GetHeight() const {
  int32_t height = 0;
  if (AImage_getHeight(aimage_, &height) != AMEDIA_OK) {
    return absl::InternalError("Unable to query the height of AImage");
  }
  return height;
}

const AHardwareBuffer* Image::GetHardwareBuffer() const {
  return ahardware_buffer_;
}

const ADataSpace Image::GetBufferDataSpace() const {
  ImageAPIProvider* api_provider = GetImageAPIProvider().get();
  if (api_provider == nullptr) {
    IMP_LOG(imp::ERROR) << "ImageAPIProvider is not set. Falling back to using dlsym.";
    return AHardwareBufferHelper::GetDataSpace(ahardware_buffer_);
  }

  int32_t data_space = 0;  // ADATASPACE_UNKNOWN
  if (absl::Status status =
          api_provider->GetBufferDataSpace(ahardware_buffer_, data_space);
      !status.ok()) {
    IMP_LOG(imp::ERROR) << "Unable to get buffer data space from ImageAPIProvider: "
               << status << ". Falling back to using dlsym.";
    return AHardwareBufferHelper::GetDataSpace(ahardware_buffer_);
  }
  return static_cast<ADataSpace>(data_space);
}

std::unique_ptr<RobinMap<SurfaceViewType, const AHardwareBuffer*>>
Image::GetViewHardwareBuffers() const {
  ImageAPIProvider* api_provider = GetImageAPIProvider().get();
  if (api_provider == nullptr) {
    IMP_LOG(imp::ERROR) << "ImageAPIProvider is not set. Falling back to using dlsym.";
    return AHardwareBufferHelper::GetAvailableViews(ahardware_buffer_);
  }

  uint32_t view_masks = 0;
  if (absl::Status status =
          api_provider->GetAuxiliaryViewInfo(ahardware_buffer_, view_masks);
      !status.ok()) {
    IMP_LOG(imp::ERROR) << "Unable to get auxiliary view info from ImageAPIProvider: "
               << status << ". Falling back to using dlsym.";
    return AHardwareBufferHelper::GetAvailableViews(ahardware_buffer_);
  }

  std::unique_ptr<RobinMap<SurfaceViewType, const AHardwareBuffer*>>
      view_hardware_buffers = absl::WrapUnique(
          new RobinMap<SurfaceViewType, const AHardwareBuffer*>());
  *view_hardware_buffers = {{SurfaceViewType::kPrimaryView, ahardware_buffer_}};
  if (!(view_masks & kQCAuxiliaryViewMask)) {
    return view_hardware_buffers;
  }

  // Retrieve the auxiliary view hardware buffer.
  AHardwareBuffer* auxiliary_buffer = nullptr;
  if (absl::Status status = api_provider->GetAuxiliaryBuffer(
          ahardware_buffer_, kQCAuxiliaryViewMask, auxiliary_buffer);
      !status.ok()) {
    IMP_LOG(imp::ERROR) << "Unable to get auxiliary view hardware buffer: " << status;
    return view_hardware_buffers;
  }

  if (auxiliary_buffer) {
    (*view_hardware_buffers)[SurfaceViewType::kAuxiliaryView] =
        auxiliary_buffer;
  } else {
    IMP_LOG(imp::ERROR) << "Auxiliary view hardware buffer was expected but is not "
                  "available.";
  }
  return view_hardware_buffers;
}

absl::StatusOr<mat4f> Image::GetTransformMatrix() const {
  float matrix[16];
  media_status_t media_status = AMEDIA_ERROR_UNKNOWN;
  absl::Status status =
      absl::UnavailableError("No API available to get transform matrix.");

  ImageAPIProvider* api_provider = GetImageAPIProvider().get();
  if (api_provider) {
    status =
        api_provider->GetImageTransformMatrix(aimage_, matrix, media_status);
  } else {
    IMP_LOG(imp::ERROR) << "Image transform matrix API is not available.";
    return absl::UnavailableError("Image transform matrix API is not enabled.");
  }

  if (!status.ok()) {
    return absl::InternalError(
        absl::StrCat("Unable to get image transform matrix. Error: ", status));
  }
  if (media_status != AMEDIA_OK) {
    return absl::InternalError(absl::StrCat(
        "Unable to get image transform matrix. media_status Error: ",
        media_status));
  }

  mat4f result;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      result[i][j] = matrix[i * 4 + j];
    }
  }

  return result;
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

}  // namespace imp::android
