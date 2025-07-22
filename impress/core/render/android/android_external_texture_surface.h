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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_

#include <array>
#include <memory>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/wrappers/surface.h"

namespace imp {

constexpr std::array<SurfaceViewType, 1>
    kAndroidExternalTextureSurfaceConfigMono = {SurfaceViewType::kPrimaryView};

constexpr std::array<SurfaceViewType, 2>
    kAndroidExternalTextureSurfaceConfigStereo = {
        SurfaceViewType::kPrimaryView, SurfaceViewType::kAuxiliaryView};

// Creates an Android Surface and corresponding external textures. If the
// Android Surface is sourced from a SurfaceTexture, only base view and its
// corresponding external texture are created. If the Android Surface is sourced
// from an ImageReader, it supports multiple views, each of which requires its
// own external texture.
class AndroidExternalTextureSurface {
 public:
  static absl::StatusOr<std::unique_ptr<AndroidExternalTextureSurface>> Create(
      BaseView& view,
      ContentSecurityLevel security_level = ContentSecurityLevel::kNone,
      absl::Span<const SurfaceViewType> view_types =
          kAndroidExternalTextureSurfaceConfigMono);

  AndroidExternalTextureSurface(AndroidExternalTextureSurface&& other) =
      default;
  AndroidExternalTextureSurface& operator=(
      AndroidExternalTextureSurface&& other) = default;

  AndroidExternalTextureSurface(const AndroidExternalTextureSurface&) = delete;
  AndroidExternalTextureSurface& operator=(
      const AndroidExternalTextureSurface&) = delete;

  // Returns the Android surface.
  android::Surface* GetSurface() const;
  absl::Status SetDefaultBufferSize(int2 size) const;
  absl::StatusOr<mat4f> GetTransformMatrix() const;

  ABSL_DEPRECATED("Use BorrowTexture instead.")
  Texture* GetTexture();

  ABSL_DEPRECATED("Use BorrowTextures instead.")
  RobinMap<SurfaceViewType, Texture*> GetTextures();

  // Borrows a single external texture for the primary view.
  BorrowedTexturePtr BorrowTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  // Borrows an external texture for each view.
  RobinMap<SurfaceViewType, BorrowedTexturePtr> BorrowTextures(
      SmallSourceLocation loc = SmallSourceLocation::Current());

  // Returns the content security level of the surface.
  ContentSecurityLevel GetContentSecurityLevel() const;

  // Returns the color space of the latest image drawn on the surface.
  absl::StatusOr<MediaColorSpace> GetMediaColorSpace() const;

 private:
  AndroidExternalTextureSurface(
      ContentSecurityLevel security_level = ContentSecurityLevel::kNone);
  absl::Status CreatePlatformSurface(
      BaseView& view, absl::Span<const SurfaceViewType> view_types);

  std::unique_ptr<PlatformAndroidExternalTextureSurface> platform_surface_;
  ContentSecurityLevel security_level_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_
