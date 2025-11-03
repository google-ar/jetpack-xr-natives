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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_ANDROID_SURFACE_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_ANDROID_SURFACE_FACTORY_H_

#include <jni.h>

#include <memory>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/robin_map.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"

namespace imp::split_engine {

// Creates and stores Android Surface objects for communicating external
// textures across the Split Engine bridge.
class SplitEngineSurfaceFactory {
 public:
  SplitEngineSurfaceFactory() = default;

  // Creates an Android surface, binds an external texture to that surface, and
  // notifies the SplitEngineRenderer of the connection between the given
  // app-side texture_id and the new renderer-side external texture.
  absl::StatusOr<jobject> CreateExternalTextureSurface(
      BaseView& view, BridgeId bridge_id,
      const std::vector<TextureId>& in_texture_ids);
  absl::Status SetExternalTextureSurfaceSize(BaseView& view, BridgeId bridge_id,
                                             TextureId texture_id, int2 size);
  void Clear(BridgeId bridge_id);

 private:
  // Returns the color space of the given surface by primary texture ID.
  MediaColorSpace GetSourceColorSpace(BridgeId bridge_id,
                                      TextureId surface_texture_id);
  jobject GetSurface(BridgeId bridge_id, TextureId surface_texture_id);

  // Releases a surface texture given the primary texture ID of the surface and
  // the texture ID of the texture to release.
  void ReleaseTexture(BridgeId bridge_id, TextureId surface_texture_id,
                      TextureId texture_id);

  // Storage for external texture surfaces.
  struct SurfaceData {
    std::unique_ptr<AndroidExternalTextureSurface> surface;
    // The set of texture IDs associated with the surface that are in-use.
    // The surface can be destroyed once the last texture is released.
    absl::flat_hash_set<TextureId> in_use_texture_ids;
  };
  RobinMap<BridgeId, RobinMap<TextureId, SurfaceData>>
      external_texture_surfaces_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_ANDROID_SURFACE_FACTORY_H_
