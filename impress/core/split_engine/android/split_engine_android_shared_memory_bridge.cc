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

#include "core/split_engine/android/split_engine_android_shared_memory_bridge.h"

#include <jni.h>

#include <cstdint>
#include <functional>
#include <vector>

#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

jobject SplitEngineAndroidSharedMemoryBridge::CreateExternalTextureSurface(
    const std::vector<TextureId>& texture_ids) {
  return split_engine_shared_memory_bridge_client_
      ->CreateExternalTextureSurface(texture_ids);
}

bool SplitEngineAndroidSharedMemoryBridge::SetExternalTextureSurfaceSize(
    uint64_t texture_id, int32_t width, int32_t height) {
  if (SplitEngineSharedMemoryBridgeClient::Result result =
          split_engine_shared_memory_bridge_client_
              ->SetExternalTextureSurfaceSize(texture_id, width, height);
      !result.is_ok()) {
    // TODO: Improve error codes returned for split engine.
    return false;
  }

  return true;
}

bool SplitEngineAndroidSharedMemoryBridge::SendRequest(
    const std::vector<uint8_t>& data,
    std::function<void(const std::vector<uint8_t>&)> callback) {
  if (SplitEngineSharedMemoryBridgeClient::Result result =
          split_engine_shared_memory_bridge_client_->SendRequest(data,
                                                                 callback);
      !result.is_ok()) {
    return false;
  }
  return true;
}

SplitEngineSharedMemoryBridgeClient&
SplitEngineAndroidSharedMemoryBridge::GetSplitEngineSharedMemoryBridgeClient() {
  return *split_engine_shared_memory_bridge_client_;
}

}  // namespace imp::split_engine
