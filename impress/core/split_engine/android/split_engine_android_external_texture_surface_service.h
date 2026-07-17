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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_ANDROID_EXTERNAL_TEXTURE_SURFACE_SERVICE_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_ANDROID_EXTERNAL_TEXTURE_SURFACE_SERVICE_H_

#include <jni.h>

#include <cstdint>
#include <vector>

#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// SplitEngineAndroidExternalTextureSurfaceService is a wrapper around the
// SplitEngineSharedMemoryBridgeClient that exposes only APIs related to
// external texture surfaces and hides all other transport-specific details.
//
class SplitEngineAndroidExternalTextureSurfaceService {
 public:
  SplitEngineAndroidExternalTextureSurfaceService(
      SplitEngineSharedMemoryBridgeClient& client)
      : client_(client) {}
  SplitEngineAndroidExternalTextureSurfaceService(
      const SplitEngineAndroidExternalTextureSurfaceService&) = delete;
  SplitEngineAndroidExternalTextureSurfaceService& operator=(
      const SplitEngineAndroidExternalTextureSurfaceService&) = delete;
  SplitEngineAndroidExternalTextureSurfaceService(
      SplitEngineAndroidExternalTextureSurfaceService&&) = default;
  SplitEngineAndroidExternalTextureSurfaceService& operator=(
      SplitEngineAndroidExternalTextureSurfaceService&&) = default;

  // Forwards the call to the SplitEngineSharedMemoryBridgeClient::
  // CreateExternalTextureSurface() to create a texture surface bound to the
  // given external texture id.
  jobject CreateExternalTextureSurface(
      const std::vector<TextureId>& texture_ids);

  // Forwards the call to the
  // SplitEngineSharedMemoryBridgeClient::SetExternalTextureSurfaceSize() to
  // set the size of an external texture surface bound to the given texture id.
  //
  // Returns true if the size was set successfully, false otherwise.
  bool SetExternalTextureSurfaceSize(TextureId texture_id, int32_t width,
                                     int32_t height);

 private:
  SplitEngineSharedMemoryBridgeClient& client_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_ANDROID_EXTERNAL_TEXTURE_SURFACE_SERVICE_H_
