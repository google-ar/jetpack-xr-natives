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

#include "core/render/android/android_external_texture_surface.h"

#include <memory>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "mediapipe/framework/port/status_macros.h"
#if IMP_PLATFORM(ANDROID)
#include "core/render/android/default_platform_android_external_texture_surface.h"
#include "core/split_engine/split_engine_serializer.h"
#if defined(IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER)
#include "core/render/android/image_reader_android_external_texture_surface.h"
#endif  // IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER
#endif  // IMP_PLATFORM(ANDROID)

namespace imp {

absl::StatusOr<std::unique_ptr<AndroidExternalTextureSurface>>
AndroidExternalTextureSurface::Create(
    BaseView& view, ContentSecurityLevel security_level,
    absl::Span<const SurfaceViewType> view_types) {
  std::unique_ptr<AndroidExternalTextureSurface> external_texture_surface =
      absl::WrapUnique(new AndroidExternalTextureSurface(security_level));
  MP_RETURN_IF_ERROR(
      external_texture_surface->CreatePlatformSurface(view, view_types));
  return external_texture_surface;
}

AndroidExternalTextureSurface::AndroidExternalTextureSurface(
    ContentSecurityLevel security_level)
    : security_level_(security_level) {}

absl::Status AndroidExternalTextureSurface::CreatePlatformSurface(
    BaseView& view, absl::Span<const SurfaceViewType> view_types) {
#if IMP_PLATFORM(ANDROID)
  if (split_engine::SplitEngineSerializer* serializer =
          view.GetSplitEngineSerializer()) {
    platform_surface_ = serializer->CreateAndroidExternalTextureSurface(
        security_level_, view_types);
  } else {
#if defined(IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER)
    if (security_level_ == ContentSecurityLevel::kNone &&
        view_types.size() == 1) {
      MP_ASSIGN_OR_RETURN(platform_surface_,
                       DefaultPlatformAndroidExternalTextureSurface::Create(
                           view, security_level_, view_types));
    } else {
      MP_ASSIGN_OR_RETURN(platform_surface_,
                       ImageReaderAndroidExternalTextureSurface::Create(
                           view, security_level_, view_types));
    }
#else   // Use SurfaceTexture (default) version.
    // TODO: The SurfaceTexture code path only supports L3 DRM.
    // We need to add support for L1 DRM before fully enabling this code
    // path for secure video playback.
    if (security_level_ == ContentSecurityLevel::kProtected) {
      return absl::InternalError(
          "Protected video playback is not supported using "
          "AndroidExternalTextureSurface with SurfaceTexture. Please use "
          "ImageReader instead.");
    }
    MP_ASSIGN_OR_RETURN(platform_surface_,
                     DefaultPlatformAndroidExternalTextureSurface::Create(
                         view, security_level_, view_types));
#endif  // IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER
  }
#endif  // IMP_PLATFORM(ANDROID)
  return absl::OkStatus();
}

android::Surface* AndroidExternalTextureSurface::GetSurface() const {
  return platform_surface_->GetSurface();
}

absl::Status AndroidExternalTextureSurface::SetDefaultBufferSize(
    int2 size) const {
  return platform_surface_->SetDefaultBufferSize(size);
}

absl::StatusOr<mat4f> AndroidExternalTextureSurface::GetTransformMatrix()
    const {
  return platform_surface_->GetTransformMatrix();
}

Texture* AndroidExternalTextureSurface::GetTexture() {
  return platform_surface_->GetTexture();
}

RobinMap<SurfaceViewType, Texture*>
AndroidExternalTextureSurface::GetTextures() {
  return platform_surface_->GetTextures();
}

BorrowedTexturePtr AndroidExternalTextureSurface::BorrowTexture(
    SmallSourceLocation loc) {
  return platform_surface_->BorrowTexture(loc);
}

RobinMap<SurfaceViewType, BorrowedTexturePtr>
AndroidExternalTextureSurface::BorrowTextures(SmallSourceLocation loc) {
  return platform_surface_->BorrowTextures(loc);
}

ContentSecurityLevel AndroidExternalTextureSurface::GetContentSecurityLevel()
    const {
  return security_level_;
}

absl::StatusOr<SurfaceColorSpace>
AndroidExternalTextureSurface::GetSurfaceColorSpace() const {
  return platform_surface_->GetSurfaceColorSpace();
}

}  // namespace imp
