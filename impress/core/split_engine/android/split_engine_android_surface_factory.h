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

#include "absl/status/status.h"
#include "core/common/robin_map.h"
#include "core/math/vec.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/texture.h"
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
  jobject CreateExternalTextureSurface(
      BaseView& view, BridgeId bridge_id,
      const std::vector<TextureId>& in_texture_ids);
  absl::Status SetExternalTextureSurfaceSize(BaseView& view, BridgeId bridge_id,
                                             TextureId texture_id, int2 size);
  void Clear(BridgeId bridge_id);

 private:
  // Storage for external texture surfaces and filament texture objects.
  struct ExternalTextureSurface {
    std::unique_ptr<AndroidExternalTextureSurface> platform_surface;
    RobinMap<SurfaceViewType, BorrowedTexturePtr> textures;
  };

  RobinMap<BridgeId, RobinMap<TextureId, ExternalTextureSurface>>
      external_texture_surfaces_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_ANDROID_SURFACE_FACTORY_H_
