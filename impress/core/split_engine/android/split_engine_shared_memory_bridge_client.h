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
#include <string>
#include <string_view>
#include <vector>

#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// An interface for a client of the SplitEngineSharedMemoryBridge.
// This interface is exchanged across shared object boundary, so the
// types should be self-contained to avoid divergent versions of the same
// objects (eg: absl::Status or flat buffer builder).
class SplitEngineSharedMemoryBridgeClient {
 public:
  virtual ~SplitEngineSharedMemoryBridgeClient() = default;

  using WorkItem = std::function<void()>;
  using WorkScheduler = std::function<void(WorkItem)>;

  // An interface for a handle to a shared memory buffer.
  class BufferHandle {
   public:
    virtual ~BufferHandle() = default;
  };

  // A self contained result class.
  class Result {
   public:
    explicit Result(bool is_ok) : is_ok_(is_ok), message_("") {}
    Result(bool is_ok, std::string message)
        : is_ok_(is_ok), message_(message) {}

    bool is_ok() const { return is_ok_; }
    std::string_view message() const { return message_; };

   private:
    const bool is_ok_;
    const std::string message_;
  };

  // Registers a shared memory buffer file descriptor with the bridge and
  // returns a handle to the shared memory buffer.
  virtual std::unique_ptr<BufferHandle> RegisterBuffer(
      int fd, size_t buffer_size_bytes) = 0;

  // Processes a region of a registered buffer specified by offset and bytes.
  virtual Result ProcessRegion(const BufferHandle& buffer_handle,
                               size_t offset_bytes,
                               size_t region_length_bytes) = 0;

  // Creates a texture surface bound to the given external texture id.
  virtual jobject CreateExternalTextureSurface(
      const std::vector<TextureId>& in_texture_ids) = 0;

  // Sets the size of an external texture surface bound to the given texture id.
  virtual Result SetExternalTextureSurfaceSize(TextureId in_texture_id,
                                               int32_t width,
                                               int32_t height) = 0;

  // Sends a flatbuffer request to the backend with a handler for a flatbuffer
  // response.
  virtual Result SendRequest(
      const std::vector<uint8_t>& data,
      std::function<void(const std::vector<uint8_t>&)> callback) = 0;

  // See ISplitEngineSharedMemoryReverseBridge.aidl for explanation of why the
  // handler's signature is void(int).
  virtual void RegisterReverseBridgeMessageHandler(
      std::function<void(int)> handler) = 0;

  // Initializes the bridge client, and pass a lambda which can be used to
  // schedule work on the client's executor.
  //
  // It allows the client to initialize internal state that is not possible
  // earlier in the lifecycle.
  virtual void Initialize(WorkScheduler work_scheduler) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_CLIENT_H_
