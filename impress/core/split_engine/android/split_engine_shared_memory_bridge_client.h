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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_CLIENT_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_CLIENT_H_

#include <jni.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// An interface for a client of the SplitEngineSharedMemoryBridge.
// This interface is exchanged across shared object boundary, so the
// types should be self-contained to avoid divergent versions of the same
// objects (eg: absl::Status or flat buffer builder).
class SplitEngineSharedMemoryBridgeClient {
 public:
  using BufferHandle = BufferHandleFactory::BufferHandle;

  virtual ~SplitEngineSharedMemoryBridgeClient() = default;

  // Returns the unique id for this client.
  virtual ClientId GetClientId() const = 0;

  // Generates a message group id for a new message group.
  // TODO: (broken link) - remove this entirely and use memory addresses.
  virtual MessageGroupId GenerateMessageGroupId() = 0;

  // Registers a shared memory buffer file descriptor with the bridge and
  // returns a handle to the shared memory buffer.
  virtual absl::StatusOr<std::unique_ptr<BufferHandle>> RegisterBuffer(
      int fd, size_t buffer_size_bytes) = 0;

  // Processes a region of a registered buffer specified by offset and bytes.
  virtual absl::Status ProcessRegion(const BufferHandle& buffer_handle,
                                     int offset_bytes,
                                     int region_length_bytes) = 0;

  // Creates a texture surface bound to the given external texture id.
  virtual absl::StatusOr<jobject> CreateExternalTextureSurface(
      const std::vector<TextureId>& in_texture_ids) = 0;

  // Sets the size of an external texture surface bound to the given texture id.
  virtual absl::Status SetExternalTextureSurfaceSize(TextureId in_texture_id,
                                                     int32_t width,
                                                     int32_t height) = 0;

  // Sends a flatbuffer request to the backend with a handler for a flatbuffer
  // response.
  virtual absl::Status SendRequest(
      const std::vector<uint8_t>& data,
      std::function<void(const std::vector<uint8_t>&)> callback) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_CLIENT_H_
