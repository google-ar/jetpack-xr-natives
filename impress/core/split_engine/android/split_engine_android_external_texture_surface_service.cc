// Copyright 2025 Google LLC
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

#include "core/split_engine/android/split_engine_android_external_texture_surface_service.h"

#include <jni.h>

#include <cstdint>
#include <vector>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

bool SplitEngineAndroidExternalTextureSurfaceService::
    SetExternalTextureSurfaceSize(TextureId texture_id, int32_t width,
                                  int32_t height) {
  return client_.SetExternalTextureSurfaceSize(texture_id, width, height).ok();
}

jobject
SplitEngineAndroidExternalTextureSurfaceService::CreateExternalTextureSurface(
    const std::vector<TextureId>& texture_ids) {
  absl::StatusOr<jobject> external_texture_surface =
      client_.CreateExternalTextureSurface(texture_ids);
  if (!external_texture_surface.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to create external texture surface with status: "
               << external_texture_surface.status().message();
    return nullptr;
  }
  return *external_texture_surface;
}

}  // namespace imp::split_engine
