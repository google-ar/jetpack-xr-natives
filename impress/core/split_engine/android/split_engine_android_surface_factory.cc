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

#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/common/invocable.h"
#include "core/common/registry.h"
#include "core/common/robin_map.h"
#include "core/config.h"
#include "core/math/vec.h"
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
jobject SplitEngineSurfaceFactory::CreateExternalTextureSurface(
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
    IMP_LOG(imp::ERROR) << "Unsupported number of texture ids provided: "
               << texture_ids.size();
    return nullptr;
  }
  absl::StatusOr<std::unique_ptr<AndroidExternalTextureSurface>>
      platform_surface = AndroidExternalTextureSurface::Create(
          view, content_security_level, view_types);
  if (!platform_surface.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to create external texture surface: "
               << platform_surface.status();
    return nullptr;
  }

  jobject surface_reference = (*platform_surface)->GetSurface()->Reference();
  if (!surface_reference) {
    IMP_LOG(imp::ERROR) << "Failed to create external texture surface: "
                  "surface_reference is null.";
    return nullptr;
  }
  RobinMap<SurfaceViewType, BorrowedTexturePtr> textures =
      (*platform_surface)->BorrowTextures();
  if (textures.empty()) {
    IMP_LOG(imp::ERROR) << "No textures created by the external texture surface.";
    return nullptr;
  }
  if (!textures.contains(SurfaceViewType::kPrimaryView)) {
    IMP_LOG(imp::ERROR) << "No texture created for primary view.";
    return nullptr;
  }
  if (textures.size() != texture_ids.size()) {
    IMP_LOG(imp::ERROR) << "Number of textures created by the external texture surface "
                  "does not match the number of texture ids provided.";
    return nullptr;
  }

  // Construct Android ExternalTextureSurface
  ExternalTextureSurface external_texture_surface;
  external_texture_surface.platform_surface = *std::move(platform_surface);
  external_texture_surface.textures = textures;
  std::function<SurfaceColorSpace()> get_source_color_space_fn =
      [surface_ptr = external_texture_surface.platform_surface.get()]() {
        absl::StatusOr<SurfaceColorSpace> surface_color_space =
            surface_ptr->GetSurfaceColorSpace();
        if (!surface_color_space.ok()) {
          IMP_LOG(imp::ERROR) << "Failed to get the surface color space";
          // Fall back to the default color space.
          return SurfaceColorSpace();
        }
        return *surface_color_space;
      };

  // Create a map of textures for each bridge.
  if (!external_texture_surfaces_.contains(bridge_id)) {
    external_texture_surfaces_.insert(
        {bridge_id, RobinMap<TextureId, ExternalTextureSurface>()});
  }
  RobinMap<TextureId, ExternalTextureSurface>& bridge_textures =
      external_texture_surfaces_.at(bridge_id);
  // Associate the external texture surface with the primary texture id on the
  // bridge. This ID represents the primary view in a single view configuration
  // or the left view in a multiview configuration.
  bridge_textures.insert({texture_ids[0], std::move(external_texture_surface)});

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
            content_security_level, get_source_color_space_fn,
            /* release_fn= */ [this, bridge_id, texture_ids, i]() {
              if (i == 0) {
                RobinMap<uint64_t, ExternalTextureSurface>& bridge_textures =
                    external_texture_surfaces_.at(bridge_id);
                bridge_textures.erase(texture_ids[i]);
              }
            });
  }
  return surface_reference;
}

absl::Status SplitEngineSurfaceFactory::SetExternalTextureSurfaceSize(
    BaseView& view, BridgeId bridge_id, TextureId texture_id, int2 size) {
  auto bridge_textures_it = external_texture_surfaces_.find(bridge_id);
  if (bridge_textures_it == external_texture_surfaces_.end()) {
    return absl::NotFoundError(
        absl::StrFormat("Bridge not found: %d", bridge_id));
  }
  const RobinMap<TextureId, ExternalTextureSurface>& bridge_textures =
      bridge_textures_it->second;
  auto texture_it = bridge_textures.find(texture_id);
  if (texture_it == bridge_textures.end()) {
    return absl::NotFoundError(
        absl::StrFormat("Texture not found: %d", texture_id));
  }
  return texture_it->second.platform_surface->SetDefaultBufferSize(size);
}

void SplitEngineSurfaceFactory::Clear(BridgeId bridge_id) {
  external_texture_surfaces_.erase(bridge_id);
}

}  // namespace imp::split_engine
