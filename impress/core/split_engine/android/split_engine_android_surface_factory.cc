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

#include "core/split_engine/android/split_engine_android_surface_factory.h"

#include <jni.h>

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "core/common/invocable.h"
#include "core/common/registry.h"
#include "core/common/robin_map.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/view/base_view.h"

// TODO: Remove this once the required unit tests are added.
namespace imp::split_engine {
absl::StatusOr<jobject> SplitEngineSurfaceFactory::CreateExternalTextureSurface(
    BaseView& view, BridgeId bridge_id,
    const std::vector<TextureId>& in_texture_ids) {
  
  absl::StatusOr<std::reference_wrapper<SplitEngineRenderer>>
      split_engine_renderer =
          view.GetRegistry().Get<split_engine::SplitEngineRenderer>();
  if (!split_engine_renderer.ok()) {
    IMP_LOG(imp::FATAL)
        << "SplitEngineSurfaceFactory requires a SplitEngineRenderer in the "
           "registry.";
  }

  const std::vector<TextureId>* texture_ids_ptr = &in_texture_ids;
  std::vector<TextureId> trimmed_texture_ids;
  ContentSecurityLevel content_security_level = ContentSecurityLevel::kNone;

  // For now we append the security level to the list of texture IDs.
  // We check the last element of the list to see if it is a security level.
  // If it is, we remove it from the list of texture IDs and use it to set the
  // security level.
  TextureId security_level_candidate = in_texture_ids.back();
  if (security_level_candidate ==
          static_cast<TextureId>(ContentSecurityLevel::kNone) ||
      security_level_candidate ==
          static_cast<TextureId>(ContentSecurityLevel::kProtected)) {
    content_security_level =
        static_cast<ContentSecurityLevel>(security_level_candidate);
    trimmed_texture_ids.assign(in_texture_ids.begin(),
                               in_texture_ids.end() - 1);
    texture_ids_ptr = &trimmed_texture_ids;
  }
  const std::vector<TextureId>& texture_ids = *texture_ids_ptr;

  absl::Span<const SurfaceViewType> view_types;
  if (texture_ids.size() == 1) {
    view_types = kAndroidExternalTextureSurfaceConfigMono;
  } else if (texture_ids.size() == 2) {
    view_types = kAndroidExternalTextureSurfaceConfigStereo;
  } else {
    return absl::InternalError(absl::StrFormat(
        "Unsupported number of texture ids provided: %d", texture_ids.size()));
  }
  absl::StatusOr<std::unique_ptr<AndroidExternalTextureSurface>>
      platform_surface = AndroidExternalTextureSurface::Create(
          view, content_security_level, view_types);
  if (!platform_surface.ok()) {
    return platform_surface.status();
  }

  jobject surface_reference = (*platform_surface)->GetSurface()->Reference();
  if (!surface_reference) {
    return absl::InternalError(
        "Failed to create external texture surface: "
        "surface_reference is null.");
  }
  RobinMap<SurfaceViewType, BorrowedTexturePtr> textures =
      (*platform_surface)->BorrowTextures();
  if (textures.empty()) {
    return absl::InternalError(
        "No textures created by the external texture surface.");
  }
  if (!textures.contains(SurfaceViewType::kPrimaryView)) {
    return absl::InternalError("No texture created for primary view.");
  }
  if (textures.size() != texture_ids.size()) {
    return absl::InternalError(
        "Number of textures created by the external texture surface does not "
        "match the number of texture ids provided.");
  }

  // Construct an entry to contain the Android ExternalTextureSurface unique
  // pointer and the set of texture IDs that are in use. This entry will be
  // moved into the map of surfaces and destroyed when the last texture is
  // released.
  SurfaceData surface_data;
  surface_data.surface = *std::move(platform_surface);
  surface_data.in_use_texture_ids.insert(texture_ids.begin(),
                                         texture_ids.end());
  TextureId surface_texture_id = texture_ids[0];
  std::function<MediaColorSpace()> get_source_color_space_fn =
      [this, bridge_id, surface_texture_id]() {
        return GetSourceColorSpace(bridge_id, surface_texture_id);
      };
  std::function<void*()> get_surface_fn = [this, bridge_id,
                                           surface_texture_id]() {
    return static_cast<void*>(GetSurface(bridge_id, surface_texture_id));
  };

  // Create a map of textures for each bridge.
  if (!external_texture_surfaces_.contains(bridge_id)) {
    external_texture_surfaces_.insert(
        {bridge_id, RobinMap<TextureId, SurfaceData>()});
  }
  RobinMap<TextureId, SurfaceData>& bridge_textures =
      external_texture_surfaces_.at(bridge_id);
  // Associate the external texture surface with the primary texture id on the
  // bridge. This ID represents the primary view in a single view configuration
  // or the left view in a multiview configuration.
  bridge_textures.insert({texture_ids[0], std::move(surface_data)});

  // Extract the texture pointers.
#if IMP_PLATFORM(ANDROID) && \
    defined(IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER)
  std::vector<BorrowedTexturePtr> texture_ptrs;
  switch (textures.size()) {
    case 1:
      texture_ptrs = {textures[SurfaceViewType::kPrimaryView]};
      break;
    case 2:
      texture_ptrs = {textures[SurfaceViewType::kPrimaryView],
                      textures[SurfaceViewType::kAuxiliaryView]};
      break;
    case 4:
      texture_ptrs = {textures[SurfaceViewType::kPrimaryView],
                      textures[SurfaceViewType::kAuxiliaryView],
                      textures[SurfaceViewType::kPrimaryViewDepth],
                      textures[SurfaceViewType::kAuxiliaryViewDepth]};
      break;
  }
#else
  std::vector<BorrowedTexturePtr> texture_ptrs = {
      textures[SurfaceViewType::kPrimaryView]};
#endif

  // Register the external textures with the SplitEngineRenderer. Tie the
  // lifetime of the external texture surface to the lifetime of the primary
  // texture.
  for (int i = 0; i < texture_ids.size(); ++i) {
    (*split_engine_renderer)
        .get()
        .SetTextureExternal(
            bridge_id, texture_ids[i], texture_ptrs[i].WithNewLocation(),
            get_source_color_space_fn, get_surface_fn,
            /* release_fn= */
            [this, bridge_id, surface_texture_id,
             texture_id = texture_ids[i]]() {
              ReleaseTexture(bridge_id, surface_texture_id, texture_id);
            });
  }
  return surface_reference;
}

MediaColorSpace SplitEngineSurfaceFactory::GetSourceColorSpace(
    BridgeId bridge_id, TextureId surface_texture_id) {
  // The bridge may have already been destroyed.
  auto it_bridge = external_texture_surfaces_.find(bridge_id);
  if (it_bridge == external_texture_surfaces_.end()) {
    return MediaColorSpace();
  }

  // The surface may have already been released (this is an error but should not
  // crash).
  auto it_surface = it_bridge->second.find(surface_texture_id);
  if (it_surface == it_bridge->second.end()) {
    IMP_LOG(imp::ERROR) << "Failed to find surface data for texture: "
               << surface_texture_id;
    return MediaColorSpace();
  }

  absl::StatusOr<MediaColorSpace> media_color_space =
      it_surface->second.surface->GetMediaColorSpace();
  if (!media_color_space.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to get the surface color space";
    // Fall back to the default color space.
    return MediaColorSpace();
  }
  return *media_color_space;
}

jobject SplitEngineSurfaceFactory::GetSurface(BridgeId bridge_id,
                                              TextureId surface_texture_id) {
  // The bridge may have already been destroyed.
  auto it_bridge = external_texture_surfaces_.find(bridge_id);
  if (it_bridge == external_texture_surfaces_.end()) {
    return nullptr;
  }

  // The surface may have already been released (this is an error but should not
  // crash).
  auto it_surface = it_bridge->second.find(surface_texture_id);
  if (it_surface == it_bridge->second.end()) {
    IMP_LOG(imp::ERROR) << "Failed to find surface data for texture: "
               << surface_texture_id;
    return nullptr;
  }

  return it_surface->second.surface->GetSurface()->WeakReference();
}

void SplitEngineSurfaceFactory::ReleaseTexture(BridgeId bridge_id,
                                               TextureId surface_texture_id,
                                               TextureId texture_id) {
  // The bridge may have already been destroyed.
  if (!external_texture_surfaces_.contains(bridge_id)) return;

  // The surface may have already been released.
  RobinMap<TextureId, SurfaceData>& bridge_textures =
      external_texture_surfaces_.at(bridge_id);
  if (!bridge_textures.contains(surface_texture_id)) {
    IMP_LOG(imp::ERROR) << "Failed to find surface data for texture: " << texture_id;
    return;
  }

  auto& surface_data = bridge_textures.at(surface_texture_id);
  surface_data.in_use_texture_ids.erase(texture_id);
  if (surface_data.in_use_texture_ids.empty()) {
    // If this is the last texture, release the surface.
    bridge_textures.erase(surface_texture_id);
  }
}

absl::Status SplitEngineSurfaceFactory::SetExternalTextureSurfaceSize(
    BaseView& view, BridgeId bridge_id, TextureId texture_id, int2 size) {
  auto bridge_textures_it = external_texture_surfaces_.find(bridge_id);
  if (bridge_textures_it == external_texture_surfaces_.end()) {
    return absl::NotFoundError(
        absl::StrFormat("Bridge not found: %d", bridge_id));
  }
  const RobinMap<TextureId, SurfaceData>& bridge_textures =
      bridge_textures_it->second;
  auto texture_it = bridge_textures.find(texture_id);
  if (texture_it == bridge_textures.end()) {
    return absl::NotFoundError(
        absl::StrFormat("Texture not found: %d", texture_id));
  }
  return texture_it->second.surface->SetDefaultBufferSize(size);
}

void SplitEngineSurfaceFactory::Clear(BridgeId bridge_id) {
  external_texture_surfaces_.erase(bridge_id);
}

}  // namespace imp::split_engine
