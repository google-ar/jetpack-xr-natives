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

#include "core/split_engine/android/split_engine_platform_android_external_texture_surface.h"

#include <android/binder_auto_utils.h>
#include <android/native_window_jni.h>
#include <jni.h>

#include <algorithm>
#include <iterator>
#include <memory>
#include <utility>
#include <vector>

#include "absl/container/inlined_vector.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/registry.h"
#include "core/common/robin_map.h"
#include "core/common/small_source_location.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

// TODO: Remove this once the required unit tests are added.
namespace imp::split_engine {

SplitEnginePlatformAndroidExternalTextureSurface::
    SplitEnginePlatformAndroidExternalTextureSurface(
        BaseView& view, ContentSecurityLevel security_level,
        absl::Span<const SurfaceViewType> view_types)
    : view_(view), security_level_(security_level) {
  // Create the external textures.
  for (SurfaceViewType view_type : view_types) {
    textures_[view_type] = view_.GetTextureFactory().CreateExternalTexture(
        {1, 1}, security_level_);
    split_engine_texture_ids_[view_type] =
        SplitEngineSerializer::GetId(textures_[view_type]->GetTexture());
  }

  // AIDL bridge only accepts a list of ordered texture IDs. Reserve an extra
  // element to store the security level.
  std::vector<TextureId> split_engine_texture_ids;
  split_engine_texture_ids.reserve(view_types.size() + 1);
  std::transform(
      view_types.begin(), view_types.end(),
      std::back_inserter(split_engine_texture_ids),
      [this](SurfaceViewType type) { return split_engine_texture_ids_[type]; });

  // For now we append the security level to the list of texture IDs.
  // TODO: Remove this once sendRequest() is implemented.
  split_engine_texture_ids.push_back(static_cast<TextureId>(security_level_));

  // Set the bridge and create the external texture surface.
  SplitEngineAndroidBridge& bridge =
      view_.GetRegistry().Get<SplitEngineAndroidBridge>()->get();

  jobject surface_object =
      bridge.CreateExternalTextureSurface(split_engine_texture_ids);
  if (!surface_object) {
    IMP_LOG(imp::FATAL) << "Failed to create Surface: surface_object is null.";
  }

  auto surface = android::Surface::Create(view_.GetContext(), surface_object,
                                          security_level_);
  if (surface.ok()) {
    surface_ = *std::move(surface);
  } else {
    IMP_LOG(imp::ERROR) << "Failed to create Surface: " << surface.status();
  }
}

Texture* SplitEnginePlatformAndroidExternalTextureSurface::GetTexture() {
  if (!surface_) {
    IMP_LOG(imp::ERROR) << "GetTexture() called before CreateSurface().";
    return nullptr;
  }

  const OwnedTexturePtr& texture = textures_[SurfaceViewType::kPrimaryView];
  if (!texture) {
    return nullptr;
  }
  return &(*texture);
}

RobinMap<SurfaceViewType, Texture*>
SplitEnginePlatformAndroidExternalTextureSurface::GetTextures() {
  RobinMap<SurfaceViewType, Texture*> textures;
  for (auto& [surface_view_type, texture_ptr] : textures_) {
    if (!texture_ptr) {
      IMP_LOG(imp::ERROR)
          << "SplitEnginePlatformAndroidExternalTextureSurface initialization "
             "error. Textures not ready for use.";
      return RobinMap<SurfaceViewType, Texture*>();
    }
    textures[surface_view_type] = &(*texture_ptr);
  }
  return textures;
}

BorrowedTexturePtr
SplitEnginePlatformAndroidExternalTextureSurface::BorrowTextureImpl(
    SmallSourceLocation loc) {
  if (!surface_) {
    IMP_LOG(imp::ERROR) << "BorrowTexture() called before CreateSurface().";
    return BorrowedTexturePtr();
  }
  return textures_[SurfaceViewType::kPrimaryView].Borrow(loc);
}

RobinMap<SurfaceViewType, BorrowedTexturePtr>
SplitEnginePlatformAndroidExternalTextureSurface::BorrowTexturesImpl(
    SmallSourceLocation loc) {
  RobinMap<SurfaceViewType, BorrowedTexturePtr> textures;
  for (auto& [surface_view_type, texture_ptr] : textures_) {
    if (!texture_ptr) {
      IMP_LOG(imp::ERROR)
          << "SplitEnginePlatformAndroidExternalTextureSurface initialization "
             "error. Textures not ready for use.";
      return RobinMap<SurfaceViewType, BorrowedTexturePtr>();
    }
    textures[surface_view_type] = texture_ptr.Borrow(loc);
  }
  return textures;
}

android::Surface* SplitEnginePlatformAndroidExternalTextureSurface::GetSurface()
    const {
  return surface_.get();
}

absl::Status
SplitEnginePlatformAndroidExternalTextureSurface::SetDefaultBufferSize(
    int2 size) const {
  SplitEngineAndroidBridge& bridge =
      view_.GetRegistry().Get<SplitEngineAndroidBridge>()->get();
  for (const auto& [surface_view_type, split_engine_texture_id] :
       split_engine_texture_ids_) {
    if (!bridge.SetExternalTextureSurfaceSize(split_engine_texture_id, size.x,
                                              size.y)) {
      return absl::InternalError("Failed to SetDefaultBufferSize");
    }
  }
  return absl::OkStatus();
}

absl::StatusOr<mat4f>
SplitEnginePlatformAndroidExternalTextureSurface::GetTransformMatrix() const {
  // Cannot get the transform matrix from the surface on the system side.
  return absl::UnimplementedError(
      "GetTransformMatrix is not implemented for "
      "SplitEnginePlatformAndroidExternalTextureSurface.");
}

ContentSecurityLevel
SplitEnginePlatformAndroidExternalTextureSurface::GetContentSecurityLevel()
    const {
  return surface_->GetContentSecurityLevel();
}

absl::StatusOr<SurfaceColorSpace>
SplitEnginePlatformAndroidExternalTextureSurface::GetSurfaceColorSpace() const {
  return absl::UnimplementedError(
      "GetSurfaceColorSpace is not implemented for "
      "SplitEnginePlatformAndroidExternalTextureSurface.");
}

}  // namespace imp::split_engine
