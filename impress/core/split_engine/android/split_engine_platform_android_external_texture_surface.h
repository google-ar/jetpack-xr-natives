
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_PLATFORM_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_PLATFORM_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_

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
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/wrappers/surface.h"

namespace imp::split_engine {

// Uses Split Engine bridge to create an Android Surface on the renderer side.
class SplitEnginePlatformAndroidExternalTextureSurface
    : public PlatformAndroidExternalTextureSurface {
 public:
  SplitEnginePlatformAndroidExternalTextureSurface(
      BaseView& view_, ContentSecurityLevel security_level,
      absl::Span<const SurfaceViewType> view_types);

  android::Surface* GetSurface() const override;
  absl::Status SetDefaultBufferSize(int2 size) const override;
  absl::StatusOr<mat4f> GetTransformMatrix() const override;

  ABSL_DEPRECATED("Use BorrowTexture instead.")
  // Creates a single external texture for the primary view.
  Texture* GetTexture() override;

  ABSL_DEPRECATED("Use BorrowTextures instead.")
  // Creates an external texture for each view.
  RobinMap<SurfaceViewType, Texture*> GetTextures() override;
  ContentSecurityLevel GetContentSecurityLevel() const override;

  absl::StatusOr<SurfaceColorSpace> GetSurfaceColorSpace() const override;

 protected:
  BorrowedTexturePtr BorrowTextureImpl(SmallSourceLocation loc) override;
  RobinMap<SurfaceViewType, BorrowedTexturePtr> BorrowTexturesImpl(
      SmallSourceLocation loc) override;

 private:
  BaseView& view_;
  std::unique_ptr<android::Surface> surface_;
  RobinMap<SurfaceViewType, OwnedTexturePtr> textures_;
  ContentSecurityLevel security_level_;
  // The ID of the raw filament::Texture* backing the TexturePtr for
  // communication with the renderer.
  RobinMap<SurfaceViewType, TextureId> split_engine_texture_ids_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_PLATFORM_ANDROID_EXTERNAL_TEXTURE_SURFACE_H_
