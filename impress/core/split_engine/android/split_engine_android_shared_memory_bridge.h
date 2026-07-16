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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_ANDROID_SHARED_MEMORY_BRIDGE_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_ANDROID_SHARED_MEMORY_BRIDGE_H_

#include <jni.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "core/common/invocable.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

/**
 * Implementation of SplitEngineAndroidBridge that uses shared memory to
 * communicate between client and service.
 *
 * This class is shared across shared object boundary, so the types should be
 * self-contained to avoid divergent versions of the same objects (eg:
 * absl::Status or flat buffer builder).
 */
class SplitEngineAndroidSharedMemoryBridge : public SplitEngineAndroidBridge {
 public:
  SplitEngineAndroidSharedMemoryBridge(
      std::unique_ptr<SplitEngineSharedMemoryBridgeClient>
          split_engine_shared_memory_bridge_client)
      : split_engine_shared_memory_bridge_client_(
            std::move(split_engine_shared_memory_bridge_client)) {}

  jobject CreateExternalTextureSurface(
      const std::vector<TextureId>& texture_ids) override;
  bool SetExternalTextureSurfaceSize(TextureId texture_id, int32_t width,
                                     int32_t height) override;
  bool SendRequest(
      absl::Span<const uint8_t> data,
      imp::Invocable<void(absl::Span<const uint8_t>)> callback) override;

 private:
  std::unique_ptr<SplitEngineSharedMemoryBridgeClient>
      split_engine_shared_memory_bridge_client_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_ANDROID_SHARED_MEMORY_BRIDGE_H_
