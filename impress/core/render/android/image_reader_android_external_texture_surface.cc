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
#include <string>
#include <utility>

#include "absl/container/inlined_vector.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/barrier.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/Platform.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/common/registry.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/render/texture_options.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/ndkwrappers/image.h"
#include "core/view/platforms/android/ndkwrappers/image_reader.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "core/view/utils/frame_time.h"
#include "core/window/shared_host_state.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

using android::ImageReader;

absl::StatusOr<std::unique_ptr<ImageReaderAndroidExternalTextureSurface>>
ImageReaderAndroidExternalTextureSurface::Create(
    BaseView& view, ContentSecurityLevel security_level,
    absl::Span<const SurfaceViewType> view_types, int2 initial_size,
    mat4f additional_transform, ADataSpace default_data_space) {
  std::unique_ptr<ImageReaderAndroidExternalTextureSurface>
      external_texture_surface =
          absl::WrapUnique(new ImageReaderAndroidExternalTextureSurface(
              view, security_level, initial_size, additional_transform,
              default_data_space));

  MP_RETURN_IF_ERROR(external_texture_surface->Initialize(view_types));
  return external_texture_surface;
}

ImageReaderAndroidExternalTextureSurface::
    ImageReaderAndroidExternalTextureSurface(
        BaseView& view, ContentSecurityLevel security_level, int2 initial_size,
        mat4f additional_transform, ADataSpace default_data_space)
    : view_(view),
      security_level_(security_level),
      latest_size_(initial_size),
      additional_transform_(additional_transform),
      default_data_space_(default_data_space) {
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
  MP_ASSIGN_OR_RETURN(image_reader_,
                   ImageReader::Create(view_, latest_size_.x, latest_size_.y,
                                       AIMAGE_FORMAT_PRIVATE, usage_flags,
                                       imp::kImageReaderBufferSize));
  MP_RETURN_IF_ERROR(image_reader_->SetImageListenerCallback(
      this, ImageReaderAndroidExternalTextureSurface::OnNewImageAvailable));

  // Create the surface backed by the ImageReader.
  MP_ASSIGN_OR_RETURN(surface_, android::Surface::Create(
                                 view_.GetContext(),
                                 image_reader_->GetSurface(), security_level_));

  for (SurfaceViewType surface_view_type : view_types) {
    if (OwnedImageReaderTexturePtr texture = ImageReaderTexture::Create(
            view_, {latest_size_.x, latest_size_.y}, security_level_);
        texture) {
      external_textures_.insert({surface_view_type, std::move(texture)});
    } else {
      return absl::InternalError("Failed to create external texture.");
    }
  }

  return absl::OkStatus();
}

Texture* ImageReaderAndroidExternalTextureSurface::GetTexture() {
  const OwnedImageReaderTexturePtr& texture =
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
  auto external_surface =
      static_cast<ImageReaderAndroidExternalTextureSurface*>(user_data);
  external_surface->NewImageAvailable();
}

absl::Status ImageReaderAndroidExternalTextureSurface::SetDefaultBufferSize(
    int2 size) const {
  return image_reader_->SetDefaultBufferSize(size);
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

  // Get the data space of the latest image.
  absl::StatusOr<ADataSpace> data_space = (*latest_image)->GetBufferDataSpace();
  if (!data_space.ok()) {
    if (!absl::IsUnavailable(data_space.status())) {
      IMP_LOG(imp::ERROR) << "Failed to get data space from the latest image: "
                 << data_space.status().ToString()
                 << ". Using default data space instead: "
                 << default_data_space_;
    }
    // Use the provided default if we cannot get the data space.
    data_space = default_data_space_;
  }

  if (*data_space != last_dataspace_) {
    MediaColorSpace color_space(data_space.value());
    MediaColorSpace last_color_space(last_dataspace_);

    IMP_LOG(imp::INFO) << "ImageReader: dataspace changed from " << last_dataspace_
               << " (" << last_color_space.ToString() << ") to "
               << data_space.value() << " (" << color_space.ToString() << ")";
    last_dataspace_ = data_space.value();
  }

  // Add the new image to the front of the deque.
  latest_images_.push_front(std::move(*latest_image));

  // Check if the ImageReader needs to be resized.
  MaybeResizeImageReader();

  // Get the supported view types and their respective native hardware buffers
  // from the latest image.
  std::unique_ptr<RobinMap<SurfaceViewType, const AHardwareBuffer*>>
      view_hardware_buffers = latest_images_.front()->GetViewHardwareBuffers();

  if (!view_hardware_buffers) {
    return absl::InternalError(
        "Failed to get view hardware buffers from the latest image.");
  }

  // Set the transform matrix for the latest acquired image.
  latest_acquired_image_transform_matrix_ =
      latest_images_.front()->GetTransformMatrix();

  mat4f accumulated_transform = kIdentityMat4f;
  if (latest_acquired_image_transform_matrix_.ok()) {
    accumulated_transform = latest_acquired_image_transform_matrix_.value();
  }
  accumulated_transform = accumulated_transform * additional_transform_;

  // Pass the transform matrix to Filament as a 3x3 matrix. Remove the
  // translation and scale components.
  mat3f transform_matrix_3f = mat3f{
      accumulated_transform[0][0], accumulated_transform[0][1],
      accumulated_transform[0][3], accumulated_transform[1][0],
      accumulated_transform[1][1], accumulated_transform[1][3],
      accumulated_transform[3][0], accumulated_transform[3][1],
      accumulated_transform[3][3],
  };

  // For each supported view, update the respective external stream by
  // setting that native hardware buffer as its underlying image.
  // TODO : Filament is investigating accepting a const void*
  // as a valid argument to setAcquiredImage(). Once that is supported, we can
  // remove the const_cast here.
  for (const auto& [surface_view_type, ahardware_buffer] :
       *view_hardware_buffers) {
    auto it = external_textures_.find(surface_view_type);
    if (it != external_textures_.end() && it->second) {
      it->second->UpdateTexture(ahardware_buffer, *data_space,
                                transform_matrix_3f);
    } else {
      IMP_LOG(imp::WARNING) << "No external texture found for surface view type: "
                   << static_cast<int>(surface_view_type);
    }
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

void ImageReaderAndroidExternalTextureSurface::MaybeResizeImageReader() {
  absl::StatusOr<int32_t> image_width = latest_images_.front()->GetWidth();
  absl::StatusOr<int32_t> image_height = latest_images_.front()->GetHeight();

  if (!image_width.ok() || !image_height.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to get Image dimensions. Width status: "
               << image_width.status()
               << ", Height status: " << image_height.status()
               << ". Using previous size instead: " << latest_size_.x << "x"
               << latest_size_.y;
    return;
  }

  const int32_t new_width = *image_width;
  const int32_t new_height = *image_height;
  if (new_width == latest_size_.x && new_height == latest_size_.y) {
    return;
  }
  if (new_width <= 0 || new_height <= 0) {
    IMP_LOG(imp::ERROR) << "Invalid image dimensions: " << new_width << "x" << new_height
               << ". Using previous size instead: " << latest_size_.x << "x"
               << latest_size_.y;
    return;
  }

  const AHardwareBuffer* buffer = latest_images_.front()->GetHardwareBuffer();
  if (!buffer) {
    IMP_LOG(imp::ERROR) << "Failed to get hardware buffer from the latest image.";
    return;
  }

  // Resize the ImageReader to the new dimensions and update the external
  // textures for RGBA images. We do not need to resize the ImageReader for
  // video formats since MediaCodec has its own way of handling changing video
  // dimensions.
  AHardwareBuffer_Desc desc;
  AHardwareBuffer_describe(buffer, &desc);
  const uint32_t format = desc.format;
  if (format != AHARDWAREBUFFER_FORMAT_R8G8B8A8_UNORM &&
      format != AHARDWAREBUFFER_FORMAT_R8G8B8X8_UNORM &&
      format != AHARDWAREBUFFER_FORMAT_R16G16B16A16_FLOAT &&
      format != AHARDWAREBUFFER_FORMAT_R10G10B10A2_UNORM) {
    return;
  }

  IMP_LOG(imp::INFO) << "Detected size change for format " << format
             << ". Resizing ImageReader from " << latest_size_.x << "x"
             << latest_size_.y << " to " << new_width << "x" << new_height;
  absl::Status resize_status =
      image_reader_->SetDefaultBufferSize({new_width, new_height});

  if (resize_status.ok()) {
    latest_size_ = {new_width, new_height};
    IMP_LOG(imp::INFO) << "ImageReader resized successfully to " << latest_size_.x
               << "x" << latest_size_.y;
  } else {
    IMP_LOG(imp::ERROR) << "Failed to resize ImageReader: " << resize_status << ". "
               << "Using previous size instead: " << latest_size_.x << "x"
               << latest_size_.y;
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
  absl::StatusOr<ADataSpace> data_space =
      latest_images_.front()->GetBufferDataSpace();
  if (!data_space.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to get data space from the latest image: "
               << data_space.status().ToString();
    data_space = ADATASPACE_UNKNOWN;
  }
  return MediaColorSpace(*data_space);
}

ImageReaderAndroidExternalTextureSurface::OwnedImageReaderTexturePtr
ImageReaderAndroidExternalTextureSurface::ImageReaderTexture::Create(
    BaseView& view, int2 size, ContentSecurityLevel security_level) {
  filament::Engine* engine = view.GetSharedEngine();

  // Create a placeholder texture until we have the first hardware buffer.
  auto texture_builder =
      filament::Texture::Builder()
          .levels(1)
          .width(size.x)
          .height(size.y)
          .format(filament::backend::TextureFormat::SRGB8_A8)
          .sampler(filament::Texture::Sampler::SAMPLER_EXTERNAL);

  if (security_level == ContentSecurityLevel::kProtected) {
    if (!filament::Texture::isProtectedTexturesSupported(*engine)) {
      IMP_LOG(imp::ERROR) << "Protected textures are not supported on this backend.";
      return {};
    }
    texture_builder.usage(filament::Texture::Usage::DEFAULT |
                          filament::Texture::Usage::PROTECTED);
  }

  filament::Texture* texture = texture_builder.build(*engine);
  if (!texture) {
    IMP_LOG(imp::ERROR) << "Could not create external texture";
    return {};
  }
  auto sampler = filament::TextureSampler(
      filament::TextureSampler::MagFilter::LINEAR,
      filament::TextureSampler::WrapMode::CLAMP_TO_EDGE);

  return absl::WrapUnique(
      new ImageReaderTexture(view, texture, sampler, security_level));
}

void ImageReaderAndroidExternalTextureSurface::ImageReaderTexture::
    OnAssignedToMaterial(const Material& material,
                         absl::string_view parameter_name,
                         UpdateTextureFn update_texture_fn) {
  // Make sure to immediately invoke the update texture function with the
  // current texture if one exists. The texture may have already been updated
  // by the time this is called.
  filament::Texture* current_texture =
      current_texture_ ? current_texture_ : texture_;
  update_texture_fn(parameter_name, current_texture);

  material_bindings_[std::make_pair(&material, std::string(parameter_name))] = {
      std::move(update_texture_fn),
      material.GetParameterTransformName(parameter_name),
  };
}

void ImageReaderAndroidExternalTextureSurface::ImageReaderTexture::
    OnUnassignedFromMaterial(const Material& material,
                             absl::string_view parameter_name) {
  material_bindings_.erase(
      std::make_pair(&material, std::string(parameter_name)));
}

void ImageReaderAndroidExternalTextureSurface::ImageReaderTexture::
    UpdateTexture(const AHardwareBuffer* buffer, ADataSpace data_space,
                  const mat3f& transform_matrix) {
  // Since the metadata is cached inside the ExternalImageHandle, we need to
  // re-register the buffer with a new handle to get the updated metadata.
  bool is_srgb = (data_space & TRANSFER_SRGB) != 0;
  filament::backend::Platform::ExternalImageHandle handle =
      window::SharedHostState::GetInstance().RegisterExternalImageHandle(
          buffer, /*sRGB=*/is_srgb);
  window::SharedHostState::ExternalImageMetadata metadata =
      window::SharedHostState::GetInstance().GetImageMetadata(handle);

  // Do not use invalid metadata, it could crash Filament.
  if (!metadata.IsValid()) {
    IMP_LOG(imp::ERROR) << "Invalid metadata for external texture: " << metadata.width
               << "x" << metadata.height
               << ", format: " << static_cast<int>(metadata.format)
               << ", usage: " << static_cast<int>(metadata.usage);
    return;
  }

  auto it = textures_.find(buffer);
  OwnedTexturePtr existing_texture = {};

  // If the texture for this buffer doesn't exist or the metadata has changed
  // or the data space has changed, create a new texture.
  if (it == textures_.end() || it->second->metadata != metadata ||
      it->second->data_space != data_space) {
    if (security_level_ == ContentSecurityLevel::kProtected) {
      metadata.usage |= filament::backend::TextureUsage::PROTECTED;
    }

    // Create an external texture for the buffer.
    OwnedTexturePtr texture = view_.GetTextureFactory().CreateExternalTexture(
        handle, {.width = metadata.width,
                 .height = metadata.height,
                 .format = metadata.format,
                 .usage = metadata.usage,
                 .sampler_options = TextureSamplerOptions{
                     .sampler_type =
                         TextureSamplerOptions::SamplerType::SAMPLER_EXTERNAL,
                 }});

    if (!texture) {
      IMP_LOG(imp::ERROR) << "Failed to create external texture for buffer " << buffer;
      return;
    }

    // Update the map with the new texture for the buffer. If there was already
    // a texture for this buffer, it will be destroyed. In order to prevent the
    // material from holding a destroyed texture in the interim, the old texture
    // is moved to the existing_texture variable.
    if (it != textures_.end()) {
      existing_texture = std::move(it->second->texture);
      textures_.erase(it);
      textures_lru_cache_.remove(buffer);
    }

    // Update the LRU cache: move this buffer to the front.
    textures_lru_cache_.push_front(buffer);

    // Limit the numbers of AHardwareBuffers held in memory, because in certain
    // scenarios the amount of buffers can grow indefinitely.
    if (textures_lru_cache_.size() > imp::kImageReaderBufferSize) {
      const AHardwareBuffer* oldest_buffer = textures_lru_cache_.back();
      textures_.erase(oldest_buffer);
      textures_lru_cache_.pop_back();
    }

    auto [new_it, inserted] = textures_.insert_or_assign(
        buffer, std::make_unique<TextureInfo>(std::move(texture), metadata,
                                              data_space, transform_matrix));
    it = new_it;
  } else {
    // Metadata and data space matched. Still update the LRU cache to move this
    // buffer to the front.
    textures_lru_cache_.remove(buffer);
    textures_lru_cache_.push_front(buffer);
  }

  it->second->transform_matrix = transform_matrix;
  current_texture_ = it->second->texture->GetTexture();

  for (const auto& [material_sampler, binding] : material_bindings_) {
    binding.update_texture_fn(material_sampler.second, current_texture_);

    const Material* material = material_sampler.first;
    if (!binding.transform_parameter_name.empty()) {
      const_cast<Material*>(material)->SetParameter(
          binding.transform_parameter_name, transform_matrix);
    }
  }
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
