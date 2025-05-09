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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_DEFAULT_PLATFORM_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_DEFAULT_PLATFORM_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_

#include <cstdint>
#include <memory>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "core/view/platforms/android/wrappers/surface_texture.h"

namespace imp {

// The standard SurfaceTexture-based video surface for Android.
//  - Creates a Filament texture to manage the native buffer backing the
//    SurfaceTexture.
//  - Filament creates a GL texture and attaches it to the GL context of the
//    SurfaceTexture.
//  - Filament keeps the GL texture updated with the latest content drawn on the
//    Android Surface by calling updateTexImage() on the renderer thread as
//    close as possible to the render time.
class DefaultPlatformAndroidExternalTextureSurface
    : public PlatformAndroidExternalTextureSurface {
 public:
  static absl::StatusOr<
      std::unique_ptr<DefaultPlatformAndroidExternalTextureSurface>>
  Create(BaseView& view, ContentSecurityLevel security_level,
         absl::Span<const SurfaceViewType> view_types);

  DefaultPlatformAndroidExternalTextureSurface(
      DefaultPlatformAndroidExternalTextureSurface&& other) = default;
  DefaultPlatformAndroidExternalTextureSurface& operator=(
      DefaultPlatformAndroidExternalTextureSurface&& other) = default;

  DefaultPlatformAndroidExternalTextureSurface(
      const DefaultPlatformAndroidExternalTextureSurface&) = delete;
  DefaultPlatformAndroidExternalTextureSurface& operator=(
      const DefaultPlatformAndroidExternalTextureSurface&) = delete;

  android::Surface* GetSurface() const override;
  absl::Status SetDefaultBufferSize(int2 size) const override;
  absl::StatusOr<mat4f> GetTransformMatrix() const override;

  ABSL_DEPRECATED("Use BorrowTexture instead.")
  Texture* GetTexture() override;
  ABSL_DEPRECATED("Use BorrowTextures instead.")
  RobinMap<SurfaceViewType, Texture*> GetTextures() override;
  ContentSecurityLevel GetContentSecurityLevel() const override;
  absl::StatusOr<SurfaceColorSpace> GetSurfaceColorSpace() const override;

 protected:
  BorrowedTexturePtr BorrowTextureImpl(SmallSourceLocation loc) override;
  RobinMap<SurfaceViewType, BorrowedTexturePtr> BorrowTexturesImpl(
      SmallSourceLocation loc) override;

 private:
  DefaultPlatformAndroidExternalTextureSurface(
      BaseView& view,
      ContentSecurityLevel security_level = ContentSecurityLevel::kNone);

  absl::Status Initialize(absl::Span<const SurfaceViewType> view_types);

  BaseView& view_;
  ContentSecurityLevel security_level_;
  std::unique_ptr<android::SurfaceTexture> surface_texture_;
  std::unique_ptr<android::Surface> surface_;
  OwnedTexturePtr texture_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_DEFAULT_PLATFORM_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_
