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

#include "core/render/android/image_reader_android_external_texture_surface.h"

#include <android/data_space.h>
#include <android/hardware_buffer.h>
#include <media/NdkImage.h>

#include <cstdint>
#include <memory>
#include <utility>

#include "absl/container/inlined_vector.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/barrier.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "core/common/registry.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/ndkwrappers/image.h"
#include "core/view/platforms/android/ndkwrappers/image_reader.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "core/view/utils/frame_time.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

using android::ImageReader;

absl::StatusOr<std::unique_ptr<ImageReaderAndroidExternalTextureSurface>>
ImageReaderAndroidExternalTextureSurface::Create(
    BaseView& view, ContentSecurityLevel security_level,
    absl::Span<const SurfaceViewType> view_types) {
  std::unique_ptr<ImageReaderAndroidExternalTextureSurface>
      external_texture_surface = absl::WrapUnique(
          new ImageReaderAndroidExternalTextureSurface(view, security_level));

  MP_RETURN_IF_ERROR(external_texture_surface->Initialize(view_types));
  return external_texture_surface;
}

ImageReaderAndroidExternalTextureSurface::
    ImageReaderAndroidExternalTextureSurface(
        BaseView& view, ContentSecurityLevel security_level)
    : view_(view), security_level_(security_level) {
  ImageReaderAndroidExternalTextureSurfaceUpdater& updater =
      view_.GetRegistry()
          .GetOrCreate<ImageReaderAndroidExternalTextureSurfaceUpdater>(view_);
  updater.AddSurface(this);
}

ImageReaderAndroidExternalTextureSurface::
    ~ImageReaderAndroidExternalTextureSurface() {
  ImageReaderAndroidExternalTextureSurfaceUpdater& updater =
      view_.GetRegistry()
          .GetOrCreate<ImageReaderAndroidExternalTextureSurfaceUpdater>(view_);
  updater.RemoveSurface(this);
}

absl::Status ImageReaderAndroidExternalTextureSurface::Initialize(
    absl::Span<const SurfaceViewType> view_types) {
  // Create an ImageReader.
  uint64_t usage_flags = AHARDWAREBUFFER_USAGE_GPU_SAMPLED_IMAGE;
  if (security_level_ == ContentSecurityLevel::kProtected) {
    usage_flags |= AHARDWAREBUFFER_USAGE_PROTECTED_CONTENT;
  }
  MP_ASSIGN_OR_RETURN(
      image_reader_,
      ImageReader::Create(view_, imp::kMaxViewWidth, imp::kMaxViewHeight,
                          AIMAGE_FORMAT_PRIVATE, usage_flags,
                          imp::kImageReaderBufferSize));

  MP_RETURN_IF_ERROR(image_reader_->SetImageListenerCallback(
      this, ImageReaderAndroidExternalTextureSurface::OnNewImageAvailable));

  // Create the surface backed by the ImageReader.
  MP_ASSIGN_OR_RETURN(surface_, android::Surface::Create(
                                 view_.GetContext(),
                                 image_reader_->GetSurface(), security_level_));

  // Create the external textures using a nullptr native stream. This creates a
  // texture with an underlying stream of type StreamType::ACQUIRED (vs.
  // StreamType::NATIVE). Later, the desired native hardware buffer is
  // connected to this stream using filament::Stream::setAcquiredImage(),
  // updating the corresponding texture's content.
  void* native_stream = nullptr;
  for (SurfaceViewType surface_view_type : view_types) {
    if (TexturePtr texture = view_.GetTextureFactory().CreateExternalTexture(
            native_stream, {1, 1}, security_level_);
        texture) {
      external_textures_.insert({surface_view_type, std::move(texture)});
    } else {
      return absl::InternalError("Failed to create external texture.");
    }
  }

  return absl::OkStatus();
}

Texture* ImageReaderAndroidExternalTextureSurface::GetTexture() {
  const OwnedTexturePtr& texture =
      external_textures_[SurfaceViewType::kPrimaryView];
  if (texture == nullptr) {
    return nullptr;
  }
  return &(*texture);
}

RobinMap<SurfaceViewType, Texture*>
ImageReaderAndroidExternalTextureSurface::GetTextures() {
  RobinMap<SurfaceViewType, Texture*> textures;
  for (const auto& [type, owned_texture] : external_textures_) {
    Texture* texture = nullptr;
    if (owned_texture != nullptr) {
      texture = &(*owned_texture);
    }
    textures[type] = texture;
  }
  return textures;
}

BorrowedTexturePtr ImageReaderAndroidExternalTextureSurface::BorrowTextureImpl(
    SmallSourceLocation loc) {
  return external_textures_[SurfaceViewType::kPrimaryView].Borrow(loc);
}

RobinMap<SurfaceViewType, BorrowedTexturePtr>
ImageReaderAndroidExternalTextureSurface::BorrowTexturesImpl(
    SmallSourceLocation loc) {
  RobinMap<SurfaceViewType, BorrowedTexturePtr> textures;
  for (const auto& [type, owned_texture] : external_textures_) {
    if (owned_texture != nullptr) {
      textures[type] = owned_texture.Borrow(loc);
    }
  }
  return textures;
}

void ImageReaderAndroidExternalTextureSurface::OnNewImageAvailable(
    void* user_data) {
  IMP_LOG(imp::INFO) << "New image is available.";
  auto external_surface =
      static_cast<ImageReaderAndroidExternalTextureSurface*>(user_data);
  external_surface->NewImageAvailable();
}

absl::Status ImageReaderAndroidExternalTextureSurface::SetDefaultBufferSize(
    int2 size) const {
  // The ImageReader doesn't directly control buffer size. It can suggest a
  // size during initialization, but ultimately relies on the image source
  // (e.g., MediaCodec) to determine the dimensions of the images it delivers.
  return absl::InternalError(
      "ImageReaderAndroidExternalTextureSurface::SetDefaultBufferSize is not "
      "supported.");
};

void ImageReaderAndroidExternalTextureSurface::NewImageAvailable() {
  absl::MutexLock lock(&is_next_image_available_mutex_);
  is_next_image_available_ = true;
}

absl::Status
ImageReaderAndroidExternalTextureSurface::AcquireAndProcessLatestImage() {
  absl::MutexLock lock(&image_acquire_mutex_);
  absl::StatusOr<std::unique_ptr<Image>> latest_image =
      image_reader_->AcquireLatestImage();
  if (!latest_image.ok()) {
    return absl::InternalError(absl::StrCat("Failed to acquire latest image: ",
                                            latest_image.status().ToString()));
  }

  if (latest_images_.size() == kMaxImagesKeptAlive) {
    // Release the oldest image.
    latest_images_.pop_back();
  }
  // Add the new image to the front of the deque.
  latest_images_.push_front(std::move(*latest_image));

  // Get the supported view types and their respective native hardware buffers
  // from the latest image.
  std::unique_ptr<RobinMap<SurfaceViewType, const AHardwareBuffer*>>
      view_hardware_buffers = latest_images_.front()->GetViewHardwareBuffers();

  // Set the transform matrix for the latest acquired image.
  latest_acquired_image_transform_matrix_ =
      latest_images_.front()->GetTransformMatrix();

  // Pass the transform matrix to Filament as a 3x3 matrix. Remove the
  // translation and scale components.
  mat3f transform_matrix_3f = kIdentityMat3f;
  if (latest_acquired_image_transform_matrix_.ok()) {
    transform_matrix_3f = mat3f{
        latest_acquired_image_transform_matrix_.value()[0][0],
        latest_acquired_image_transform_matrix_.value()[0][1],
        latest_acquired_image_transform_matrix_.value()[0][3],
        latest_acquired_image_transform_matrix_.value()[1][0],
        latest_acquired_image_transform_matrix_.value()[1][1],
        latest_acquired_image_transform_matrix_.value()[1][3],
        latest_acquired_image_transform_matrix_.value()[3][0],
        latest_acquired_image_transform_matrix_.value()[3][1],
        latest_acquired_image_transform_matrix_.value()[3][3],
    };
  }

  // For each supported view, update the respective external stream by
  // setting that native hardware buffer as its underlying image.
  // TODO : Filament is investigating accepting a const void*
  // as a valid argument to setAcquiredImage(). Once that is supported, we can
  // remove the const_cast here.
  for (const auto& [surface_view_type, ahardware_buffer] :
       *view_hardware_buffers) {
    external_textures_[surface_view_type]->GetStream()->setAcquiredImage(
        const_cast<void*>(static_cast<const void*>(ahardware_buffer)),
        [](void* ahardware_buffer, void* user_data) {}, nullptr,
        transform_matrix_3f);
  }

  return absl::OkStatus();
}

void ImageReaderAndroidExternalTextureSurface::Update() {
  // This method is called just before the renderer is about to render a
  // frame. If the next image is available, we acquire the latest image and
  // update the external streams. Otherwise, we keep the streams attached to
  // the previous image.
  {
    absl::MutexLock lock(&is_next_image_available_mutex_);
    if (!is_next_image_available_) return;
    is_next_image_available_ = false;
  }

  // Acquire and consume the latest image. If this fails, the renderer continues
  // to render the last successfully acquired image.
  if (absl::Status status = AcquireAndProcessLatestImage(); !status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to acquire and process the latest image: " << status;
  }
}

android::Surface* ImageReaderAndroidExternalTextureSurface::GetSurface() const {
  return surface_.get();
}

ContentSecurityLevel
ImageReaderAndroidExternalTextureSurface::GetContentSecurityLevel() const {
  return security_level_;
}

absl::StatusOr<mat4f>
ImageReaderAndroidExternalTextureSurface::GetTransformMatrix() const {
  return latest_acquired_image_transform_matrix_;
}

absl::StatusOr<MediaColorSpace>
ImageReaderAndroidExternalTextureSurface::GetMediaColorSpace() const {
  absl::MutexLock lock(&image_acquire_mutex_);
  if (latest_images_.empty()) {
    return absl::InternalError("No images available to extract color space.");
  }
  ADataSpace data_space = latest_images_.front()->GetBufferDataSpace();
  return MediaColorSpace(data_space);
}

ImageReaderAndroidExternalTextureSurfaceUpdater::
    ImageReaderAndroidExternalTextureSurfaceUpdater(BaseView& view)
    : Updater(view) {}

void ImageReaderAndroidExternalTextureSurfaceUpdater::AddSurface(
    ImageReaderAndroidExternalTextureSurface* surface) {
  surfaces_.insert(surface);
}

void ImageReaderAndroidExternalTextureSurfaceUpdater::RemoveSurface(
    ImageReaderAndroidExternalTextureSurface* surface) {
  surfaces_.erase(surface);
}

void ImageReaderAndroidExternalTextureSurfaceUpdater::Update(
    const FrameTime& frame_time) {
  for (ImageReaderAndroidExternalTextureSurface* surface : surfaces_) {
    surface->Update();
  }
}

}  // namespace imp
