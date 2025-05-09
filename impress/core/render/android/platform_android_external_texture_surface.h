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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_PLATFORM_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_PLATFORM_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/owned_ptr.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/android/android_defines.h"
#include "core/render/content_security_level.h"
#include "core/video/video_color_space.h"
#include "core/view/platforms/android/wrappers/surface.h"

namespace imp {
class Texture;
using BorrowedTexturePtr = BorrowedPtr<Texture>;

using SurfaceColorSpace = video::VideoColorSpace;
}  // namespace imp

namespace imp {
// Interface for creating an Android Surface and corresponding external texture.
class PlatformAndroidExternalTextureSurface {
 public:
  virtual ~PlatformAndroidExternalTextureSurface() = default;

  virtual android::Surface* GetSurface() const = 0;
  virtual absl::Status SetDefaultBufferSize(int2 size) const = 0;
  virtual absl::StatusOr<mat4f> GetTransformMatrix() const = 0;

  ABSL_DEPRECATED("Use BorrowTexture instead.")
  // Creates a single external texture for the primary view.
  virtual Texture* GetTexture() = 0;

  ABSL_DEPRECATED("Use BorrowTextures instead.")
  // Creates an external texture for each view.
  virtual RobinMap<SurfaceViewType, Texture*> GetTextures() = 0;

  virtual absl::StatusOr<SurfaceColorSpace> GetSurfaceColorSpace() const = 0;

  // Borrows a single external texture for the primary view.
  BorrowedTexturePtr BorrowTexture(
      SmallSourceLocation loc = SmallSourceLocation::Current()) {
    return BorrowTextureImpl(loc);
  };

  // Borrows an external texture for each view.
  RobinMap<SurfaceViewType, BorrowedTexturePtr> BorrowTextures(
      SmallSourceLocation loc = SmallSourceLocation::Current()) {
    return BorrowTexturesImpl(loc);
  };

 protected:
  virtual BorrowedTexturePtr BorrowTextureImpl(SmallSourceLocation loc) = 0;

  virtual RobinMap<SurfaceViewType, BorrowedTexturePtr> BorrowTexturesImpl(
      SmallSourceLocation loc) = 0;

  // Returns the content security level of the surface.
  virtual ContentSecurityLevel GetContentSecurityLevel() const = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_ANDROID_PLATFORM_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_
