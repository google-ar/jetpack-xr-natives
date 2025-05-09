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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_CLIENT_NDK_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_CLIENT_NDK_H_

#include <aidl/imp/split_engine/ISplitEngineSharedMemoryBridge.h>
#include <aidl/imp/split_engine/ISplitEngineSharedMemoryReverseBridge.h>
#include <android/binder_auto_utils.h>
#include <android/binder_ibinder.h>
#include <jni.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// An NDK specific implementation of the `SplitEngineSharedMemoryBridgeClient`.
class SplitEngineSharedMemoryBridgeClientNdk
    : public SplitEngineSharedMemoryBridgeClient {
 public:
  // `bridge_service_handle` is an IBinder referring to the
  // SplitEngineSharedMemoryBridgeService on the receiving end, the service
  // that receives and processes the sent messages.
  SplitEngineSharedMemoryBridgeClientNdk(
      const ndk::SpAIBinder& bridge_service_handle, JNIEnv* jni_env);

  std::unique_ptr<BufferHandle> RegisterBuffer(
      int fd, size_t buffer_size_bytes) override;

  Result ProcessRegion(const BufferHandle& buffer_handle, size_t offset_bytes,
                       size_t region_length_bytes) override;

  jobject CreateExternalTextureSurface(
      const std::vector<TextureId>& in_texture_ids) override;

  Result SetExternalTextureSurfaceSize(TextureId in_texture_id, int32_t width,
                                       int32_t height) override;

  Result SendRequest(
      const std::vector<uint8_t>& data,
      std::function<void(const std::vector<uint8_t>&)> callback) override;

  void RegisterReverseBridgeMessageHandler(
      std::function<void(int)> handler) override;

  void Initialize(WorkScheduler work_scheduler) override;

  ndk::SpAIBinder GetBridgeHandle() const { return bridge_handle_; };

 private:
  // Wraps an NDK IBinder handle to a buffer in an implementation independent
  // interface.
  class NdkBufferHandle
      : public SplitEngineSharedMemoryBridgeClient::BufferHandle {
   public:
    explicit NdkBufferHandle(ndk::SpAIBinder buffer_handle)
        : buffer_handle_(buffer_handle) {};

    const ndk::SpAIBinder buffer_handle_;
  };

  std::shared_ptr<aidl::imp::split_engine::ISplitEngineSharedMemoryBridge>
      bridge_service_;
  std::shared_ptr<
      aidl::imp::split_engine::ISplitEngineSharedMemoryReverseBridge>
      reverse_bridge_;
  // An IBinder referring to the communication channel, i.e. the bridge
  // itself. Not to be confused with `bridge_service_handle` received in the
  // constructor, which is the IBinder referring to the receiving service.
  ndk::SpAIBinder bridge_handle_;
  JavaVM* java_vm_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_CLIENT_NDK_H_
