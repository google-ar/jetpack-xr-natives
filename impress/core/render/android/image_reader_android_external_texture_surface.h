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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_IMAGE_READER_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_IMAGE_READER_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_

#include <android/data_space.h>
#include <android/hardware_buffer.h>

#include <deque>
#include <list>
#include <memory>
#include <string>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/Platform.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/common/owned_ptr.h"
#include "core/common/robin_map.h"
#include "core/common/robin_set.h"
#include "core/common/small_source_location.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/ncsb/update_phase.h"
#include "core/ncsb/update_system.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/ndkwrappers/image.h"
#include "core/view/platforms/android/ndkwrappers/image_reader.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "core/view/utils/frame_time.h"
#include "core/window/shared_host_state.h"

namespace imp {

using android::Image;
using android::ImageReader;

// This class manages an Android Surface sourced from an ImageReader and
// the external textures associated to the supported views.
//  - Supports single-view (standard) and multi-view (spatial) use cases.
//  - Creates one external texture per view (or depth).
//  - Tracks the new Image availability and updates the external textures
//    just before rendering the next frame. The update is dobe by binding
//    the respective native hardware buffers of the Image to the underlying
//    Filament streams of the external textures.
class ImageReaderAndroidExternalTextureSurface
    : public PlatformAndroidExternalTextureSurface {
 public:
  static absl::StatusOr<
      std::unique_ptr<ImageReaderAndroidExternalTextureSurface>>
  Create(BaseView& view, ContentSecurityLevel security_level,
         absl::Span<const SurfaceViewType> view_types);

  ImageReaderAndroidExternalTextureSurface(
      ImageReaderAndroidExternalTextureSurface&& other) = default;
  ImageReaderAndroidExternalTextureSurface& operator=(
      ImageReaderAndroidExternalTextureSurface&& other) = default;

  ImageReaderAndroidExternalTextureSurface(
      const ImageReaderAndroidExternalTextureSurface&) = delete;
  ImageReaderAndroidExternalTextureSurface& operator=(
      const ImageReaderAndroidExternalTextureSurface&) = delete;

  ~ImageReaderAndroidExternalTextureSurface() override;

  // Update the Filament streams at the very end just before rendering the
  // frame.
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kEnd;

  android::Surface* GetSurface() const override;

  ABSL_DEPRECATED("Use BorrowTexture instead.")
  Texture* GetTexture() override;
  ABSL_DEPRECATED("Use BorrowTextures instead.")
  RobinMap<SurfaceViewType, Texture*> GetTextures() override;

  // The ImageReader doesn't directly control buffer size. It can suggest a size
  // during initialization, but ultimately relies on the image source (e.g.,
  // MediaCodec) to determine the dimensions of the images it delivers. The
  // implementation of this method is a no-op.
  absl::Status SetDefaultBufferSize(int2 size) const override;

  // Updates the Filament streams at the very end just before rendering the
  // frame.
  void Update();

  absl::StatusOr<mat4f> GetTransformMatrix() const override;

  absl::StatusOr<MediaColorSpace> GetMediaColorSpace() const override;

 protected:
  BorrowedTexturePtr BorrowTextureImpl(SmallSourceLocation loc) override;
  RobinMap<SurfaceViewType, BorrowedTexturePtr> BorrowTexturesImpl(
      SmallSourceLocation loc) override;

  ContentSecurityLevel GetContentSecurityLevel() const override;

 private:
  ImageReaderAndroidExternalTextureSurface(
      BaseView& view,
      ContentSecurityLevel security_level = ContentSecurityLevel::kNone);

  // A texture that can be bound to multiple hardware buffers.
  // This is used to update the underlying hardware buffer of the texture
  // without creating a new texture. It overrides the imp::Texture
  // OnAssignedToMaterial and OnUnassignedFromMaterial functions to know when it
  // is bound to a material so it can update the texture used by the material.
  //
  // Internally, it keeps track of the texture that should be used by the
  // material. When the texture is updated, it traverses all the materials that
  // are using the texture and updates their samplers with the new texture. It
  // holds a map from Android hardware buffers to a texture and associated
  // metadata to avoid unnecessarily creating new textures.
  class ImageReaderTexture;
  using OwnedImageReaderTexturePtr = OwnedPtr<ImageReaderTexture>;
  class ImageReaderTexture : public Texture {
   public:
    static OwnedImageReaderTexturePtr Create(
        BaseView& view, int2 size, ContentSecurityLevel security_level);

    void OnAssignedToMaterial(const Material& material,
                              absl::string_view parameter_name,
                              UpdateTextureFn update_texture_fn) override;
    void OnUnassignedFromMaterial(const Material& material,
                                  absl::string_view parameter_name) override;

    void UpdateTexture(const AHardwareBuffer* buffer, ADataSpace data_space,
                       const mat3f& transform_matrix);

   private:
    ImageReaderTexture(BaseView& view, filament::Texture* texture,
                       filament::TextureSampler sampler,
                       ContentSecurityLevel security_level)
        : Texture(view, nullptr, texture, sampler, security_level),
          current_texture_(texture) {}

    struct TextureInfo {
      TextureInfo(OwnedTexturePtr texture,
                  window::SharedHostState::ExternalImageMetadata metadata,
                  ADataSpace data_space, mat3f transform_matrix)
          : texture(std::move(texture)),
            metadata(metadata),
            data_space(data_space),
            transform_matrix(transform_matrix) {}
      OwnedTexturePtr texture;
      window::SharedHostState::ExternalImageMetadata metadata;
      ADataSpace data_space;
      mat3f transform_matrix;
    };
    // A map from Android hardware buffers to a texture and associated metadata.
    absl::flat_hash_map<const AHardwareBuffer*, std::unique_ptr<TextureInfo>>
        textures_;
    // List of Android hardware buffers that have been used to create textures.
    // This is used to limit the number of textures to the buffer size.
    std::list<const AHardwareBuffer*> textures_lru_cache_;
    // The texture that should be used by the material. This is updated when
    // a new image is available from the ImageReader stream.
    filament::Texture* current_texture_;

    // A binding to a specific material parameter.
    struct MaterialBinding {
      UpdateTextureFn update_texture_fn;
      absl::string_view transform_parameter_name;
    };
    // A map of material parameter bindings that use this texture. The key is
    // a pair of <Material, parameter_name>, allowing this texture to be bound
    // to multiple parameters across multiple materials.
    absl::flat_hash_map<std::pair<const Material*, std::string>,
                        MaterialBinding>
        material_bindings_;
  };

  // Callback to be invoked when a new Image is available in the ImageReader.
  // The signature conforms to ImageReader::ImageListenerCallback. The user data
  // is a pointer to the ImageReaderAndroidExternalTextureSurface instance that
  // owns the ImageReader.
  static void OnNewImageAvailable(void* user_data)
      ABSL_LOCKS_EXCLUDED(is_next_image_available_mutex_);

  // Initializes the external texture surface for the given view types.
  absl::Status Initialize(absl::Span<const SurfaceViewType> view_types);

  // Handles the arrival of a new Image.
  void NewImageAvailable() ABSL_LOCKS_EXCLUDED(is_next_image_available_mutex_);

  // Acquires the latest Image from the ImageReader and processes it.
  absl::Status AcquireAndProcessLatestImage();

  BaseView& view_;
  std::unique_ptr<android::Surface> surface_{nullptr};

  // ImageReader instance used to create the Android surface.
  std::unique_ptr<ImageReader> image_reader_{nullptr};

  // External textures created for each supported view.
  RobinMap<SurfaceViewType, OwnedImageReaderTexturePtr> external_textures_;

  // Indicates whether the ImageReader has provided a new Image that is ready
  // for processing. This flag is set to true by the callback and reset to false
  // when the Image is acquired for processing.
  mutable absl::Mutex is_next_image_available_mutex_;

  // Mutex to guard the acquisition of the latest image.
  mutable absl::Mutex image_acquire_mutex_;

  bool is_next_image_available_
      ABSL_GUARDED_BY(is_next_image_available_mutex_) = false;

  // Latest images acquired from the underlying ImageReader.
  std::deque<std::unique_ptr<Image>> latest_images_
      ABSL_GUARDED_BY(image_acquire_mutex_);

  // The security level of the external textures.
  const ContentSecurityLevel security_level_;

  absl::StatusOr<mat4f> latest_acquired_image_transform_matrix_ =
      absl::UnavailableError("No transform matrix available.");

  // Test accessor.
  friend class ImageReaderAndroidExternalTextureSurfaceTestPeer;
};

// An updater that calls Update() on all image reader surfaces in the view.
class ImageReaderAndroidExternalTextureSurfaceUpdater
    : public UpdateSystem::Updater<
          ImageReaderAndroidExternalTextureSurfaceUpdater> {
 public:
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kEnd;

  explicit ImageReaderAndroidExternalTextureSurfaceUpdater(BaseView& view);

  void AddSurface(ImageReaderAndroidExternalTextureSurface* surface);
  void RemoveSurface(ImageReaderAndroidExternalTextureSurface* surface);
  void Update(const FrameTime& frame_time) override;

 private:
  // Set of SplitEngineMaterials whose parameters have changed this frame.
  RobinSet<ImageReaderAndroidExternalTextureSurface*> surfaces_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_IMAGE_READER_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_
