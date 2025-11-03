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
#include <utility>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/notification.h"
#include "absl/types/span.h"
#include "core/async/executor.h"
#include "core/async/future.h"
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
#include "core/view/base_view.h"
#include "core/view/platforms/android/wrappers/surface.h"
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
  absl::Notification notification;
  absl::Status surface_status;
  // Note: Normally we don't need to get the absl::Status, but we need to handle
  // the failure case as well to trigger notification.

  // TL;DR: All subsequent .Then() calls in this chain must use the background
  // executor to prevent a deadlock.
  //
  // Walkthrough of the deadlock scenario:
  // 1) We schedule CreatePlatformSurface() on the foreground executor.
  // 2) This eventually calls into a platform-specific implementation, for
  //    example, SplitEnginePlatformAndroidExternalTextureSurface::Create().
  // 3) The Split Engine implementation schedules the actual surface creation
  //    work on a *background* thread.
  // 4) The foreground thread now blocks at notification.WaitForNotification(),
  //    waiting for the background work to complete. Because the foreground
  //    thread is blocked, any .Then() callbacks that use the default foreground
  //    executor will never run, causing a deadlock.
  // 5) To avoid this, all chained .Then() calls must explicitly specify
  //    Executor::Type::kBackground.
  // 6) The background thread eventually finishes, calls notification.Notify(),
  //    and unblocks the foreground thread.
  external_texture_surface->CreatePlatformSurface(view, view_types)
      .Then(
          [&notification, &surface_status](absl::Status status) {
            surface_status = status;
            notification.Notify();
          },
          Executor::Type::kBackground)
      .KeptBy(&view);
  notification.WaitForNotification();

  if (!surface_status.ok()) {
    return surface_status;
  }
  return external_texture_surface;
}

Future<std::unique_ptr<AndroidExternalTextureSurface>>
AndroidExternalTextureSurface::CreateAsync(
    BaseView& view, ContentSecurityLevel security_level,
    absl::Span<const SurfaceViewType> view_types) {
  std::unique_ptr<AndroidExternalTextureSurface> external_texture_surface =
      absl::WrapUnique(new AndroidExternalTextureSurface(security_level));
  return external_texture_surface->CreatePlatformSurface(view, view_types)
      .Then([external_texture_surface =
                 std::move(external_texture_surface)]() mutable {
        return std::move(external_texture_surface);
      });
}

AndroidExternalTextureSurface::AndroidExternalTextureSurface(
    ContentSecurityLevel security_level)
    : security_level_(security_level) {}

Future<absl::Status> AndroidExternalTextureSurface::CreatePlatformSurface(
    BaseView& view, absl::Span<const SurfaceViewType> view_types) {
#if IMP_PLATFORM(ANDROID)
  if (split_engine::SplitEngineSerializer* serializer =
          view.GetSplitEngineSerializer()) {
    return serializer
        ->CreateAndroidExternalTextureSurface(security_level_, view_types)
        .Then(
            [this](std::unique_ptr<PlatformAndroidExternalTextureSurface>
                       surface) {
              platform_surface_ = std::move(surface);
              return absl::OkStatus();
            },
            Executor::Type::kBackground);
  } else {
#if defined(IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER)
    bool use_surface_texture = security_level_ == ContentSecurityLevel::kNone &&
                               view_types.size() == 1;

#if IMP_MATERIAL_API(VULKAN)
    // SurfaceTexture is not supported for Vulkan, so always use ImageReader.
    use_surface_texture = false;
#endif  // IMP_MATERIAL_API(VULKAN)

    if (use_surface_texture) {
      absl::StatusOr<
          std::unique_ptr<DefaultPlatformAndroidExternalTextureSurface>>
          platform_surface =
              DefaultPlatformAndroidExternalTextureSurface::Create(
                  view, security_level_, view_types);
      if (!platform_surface.ok()) {
        return Future<absl::Status>(platform_surface.status());
      }
      platform_surface_ = std::move(*platform_surface);

    } else {
      absl::StatusOr<std::unique_ptr<ImageReaderAndroidExternalTextureSurface>>
          platform_surface = ImageReaderAndroidExternalTextureSurface::Create(
              view, security_level_, view_types);
      if (!platform_surface.ok()) {
        return Future<absl::Status>(platform_surface.status());
      }
      platform_surface_ = std::move(*platform_surface);
    }
    return Future<absl::Status>(absl::OkStatus());
#else  // Use SurfaceTexture (default) version.

#if IMP_MATERIAL_API(VULKAN)
    return Future<absl::Status>(absl::InternalError(
        "SurfaceTexture is not supported for Vulkan. Please use ImageReader "
        "instead."));
#else
    // TODO: The SurfaceTexture code path only supports L3 DRM.
    // We need to add support for L1 DRM before fully enabling this code
    // path for secure video playback.
    if (security_level_ == ContentSecurityLevel::kProtected) {
      return Future<absl::Status>(absl::InternalError(
          "Protected video playback is not supported using "
          "AndroidExternalTextureSurface with SurfaceTexture. Please use "
          "ImageReader instead."));
    }
    absl::StatusOr<std::unique_ptr<PlatformAndroidExternalTextureSurface>>
        platform_surface = DefaultPlatformAndroidExternalTextureSurface::Create(
            view, security_level_, view_types);
    if (!platform_surface.ok()) {
      return Future<absl::Status>(platform_surface.status());
    }
    platform_surface_ = std::move(*platform_surface);
#endif  // IMP_MATERIAL_API(VULKAN)

#endif  // IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER
  }
#endif  // IMP_PLATFORM(ANDROID)
  return Future<absl::Status>(absl::OkStatus());
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

absl::StatusOr<MediaColorSpace>
AndroidExternalTextureSurface::GetMediaColorSpace() const {
  return platform_surface_->GetMediaColorSpace();
}

}  // namespace imp
