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

#include "core/render/android/default_platform_android_external_texture_surface.h"

#include <sys/types.h>

#include <memory>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "core/view/platforms/android/wrappers/surface_texture.h"
#include "mediapipe/framework/port/status_macros.h"

#if IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)
#include <GLES2/gl2.h>
#endif  // IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)

namespace imp {

absl::StatusOr<std::unique_ptr<DefaultPlatformAndroidExternalTextureSurface>>
DefaultPlatformAndroidExternalTextureSurface::Create(
    BaseView& view, ContentSecurityLevel security_level,
    absl::Span<const SurfaceViewType> view_types) {
  std::unique_ptr<DefaultPlatformAndroidExternalTextureSurface>
      external_texture_surface =
          absl::WrapUnique(new DefaultPlatformAndroidExternalTextureSurface(
              view, security_level));

  MP_RETURN_IF_ERROR(external_texture_surface->Initialize(view_types));
  return external_texture_surface;
}

DefaultPlatformAndroidExternalTextureSurface::
    DefaultPlatformAndroidExternalTextureSurface(
        BaseView& view, ContentSecurityLevel security_level)
    : view_(view), security_level_(security_level) {}

absl::Status DefaultPlatformAndroidExternalTextureSurface::Initialize(
    absl::Span<const SurfaceViewType> view_types) {
  if (view_types.size() != 1 ||
      view_types[0] != SurfaceViewType::kPrimaryView) {
    return absl::InternalError(
        "DefaultPlatformAndroidExternalTextureSurface only supports a single "
        "view.");
  }
  if (security_level_ == ContentSecurityLevel::kProtected) {
    // Generate a valid texture ID for the SurfaceTexture. This is required for
    // the SurfaceTexture to support detachment from the unprotected context and
    // attachment to the protected context.
#if IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)
    uint32_t texture_id = 0;
    glGenTextures(1, &texture_id);
    surface_texture_ = std::make_unique<android::SurfaceTexture>(
        view_.GetContext(), texture_id, /* is_secure= */ true);
#else
    return absl::InternalError(
        "Protected content is not supported on this platform.");
#endif  // IMP_PLATFORM(ANDROID) && IMP_MATERIAL_API(OPENGL)
  } else {
    // Construct Android SurfaceTexture and Surface
    surface_texture_ =
        std::make_unique<android::SurfaceTexture>(view_.GetContext(), 0);
  }

  MP_ASSIGN_OR_RETURN(
      surface_, android::Surface::Create(view_.GetContext(), *surface_texture_,
                                         security_level_));

  // Create the external texture with an arbitrary size.
  if (texture_ = view_.GetTextureFactory().CreateExternalTexture(
          surface_texture_->WeakReference(), {1, 1}, security_level_);
      !texture_) {
    return absl::InternalError("Failed to create external texture.");
  }
  return absl::OkStatus();
}

android::Surface* DefaultPlatformAndroidExternalTextureSurface::GetSurface()
    const {
  return surface_.get();
}

absl::Status DefaultPlatformAndroidExternalTextureSurface::SetDefaultBufferSize(
    int2 size) const {
  return surface_texture_->SetDefaultBufferSize(size);
}

absl::StatusOr<mat4f>
DefaultPlatformAndroidExternalTextureSurface::GetTransformMatrix() const {
  return surface_texture_->GetTransformMatrix();
}

Texture* DefaultPlatformAndroidExternalTextureSurface::GetTexture() {
  if (texture_ == nullptr) {
    return nullptr;
  }
  return &(*texture_);
}

RobinMap<SurfaceViewType, Texture*>
DefaultPlatformAndroidExternalTextureSurface::GetTextures() {
  RobinMap<SurfaceViewType, Texture*> textures;
  Texture* texture = nullptr;
  if (texture_ != nullptr) {
    texture = &(*texture_);
  }
  textures[SurfaceViewType::kPrimaryView] = texture;
  return textures;
}

BorrowedTexturePtr
DefaultPlatformAndroidExternalTextureSurface::BorrowTextureImpl(
    SmallSourceLocation loc) {
  return texture_.Borrow(loc);
}

RobinMap<SurfaceViewType, BorrowedTexturePtr>
DefaultPlatformAndroidExternalTextureSurface::BorrowTexturesImpl(
    SmallSourceLocation loc) {
  RobinMap<SurfaceViewType, BorrowedTexturePtr> textures;
  textures[SurfaceViewType::kPrimaryView] = texture_.Borrow(loc);
  return textures;
}

ContentSecurityLevel
DefaultPlatformAndroidExternalTextureSurface::GetContentSecurityLevel() const {
  return security_level_;
}

absl::StatusOr<MediaColorSpace>
DefaultPlatformAndroidExternalTextureSurface::GetMediaColorSpace() const {
#if IMP_PLATFORM(ANDROID)
#if __ANDROID_API__ >= 33
  int32_t data_space = surface_texture_->GetDataSpace();
  return MediaColorSpace(data_space);
#else
  return absl::UnimplementedError(
      "GetMediaColorSpace for "
      "DefaultPlatformAndroidExternalTextureSurface requires Android API level "
      "33 or higher.");
#endif  // __ANDROID_API__ >= 33
#else
  return absl::UnimplementedError(
      "GetMediaColorSpace is not implemented for "
      "DefaultPlatformAndroidExternalTextureSurface on this platform.");
#endif  // IMP_PLATFORM(ANDROID)
}

}  // namespace imp
