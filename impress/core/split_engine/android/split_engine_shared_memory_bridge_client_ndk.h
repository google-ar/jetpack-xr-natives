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

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/common/invocable.h"
#include "core/split_engine/android/extensions/split_engine_bridge.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// The version of the Split Engine client for non-AndroidXR environments.
//
// Note: SplitEngineSharedMemoryBridgeClientNdk is never actually used by its
// base class - the only reason it is a subclass is to signal method parity
// between SplitEngineBridge and SplitEngineSharedMemoryBridgeClientNdk which
// communicate via a 1:1 API across the JNI boundary.
class SplitEngineSharedMemoryBridgeClientNdk
    : public SplitEngineSharedMemoryBridgeClient {
 public:
  // `bridge_service_handle` is an IBinder referring to the
  // SplitEngineSharedMemoryBridgeService on the receiving end, the service
  // that receives and processes the sent messages.
  SplitEngineSharedMemoryBridgeClientNdk(
      const ndk::SpAIBinder& bridge_service_handle, JNIEnv* jni_env);

  // Note: GetClientId and GenerateMessageGroupId are only used by the client
  // "front end", i.e. SplitEngineBridge. Both of these are unimplemented and
  // should never be called.
  ClientId GetClientId() const override;
  MessageGroupId GenerateMessageGroupId() override;

  // Registers a message group callback to be invoked when the message group is
  // complete.
  void RegisterMessageGroupCallback(
      std::unique_ptr<SplitEngineMessageGroupCallback> callback);

  absl::StatusOr<
      std::unique_ptr<SplitEngineSharedMemoryBridgeClient::BufferHandle>>
  RegisterBuffer(int fd, size_t buffer_size_bytes) override;

  absl::Status ProcessRegion(
      const SplitEngineSharedMemoryBridgeClient::BufferHandle& buffer_handle,
      int offset_bytes, int region_length_bytes) override;

  absl::StatusOr<jobject> CreateExternalTextureSurface(
      const std::vector<TextureId>& in_texture_ids) override;

  absl::Status SetExternalTextureSurfaceSize(TextureId in_texture_id,
                                             int32_t width,
                                             int32_t height) override;

  absl::Status SendRequest(
      absl::Span<const uint8_t> data,
      imp::Invocable<void(absl::Span<const uint8_t>)> callback) override;

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
  std::unique_ptr<SplitEngineMessageGroupCallback>
      release_message_group_callback_;
  int next_message_group_id_ = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_SPLIT_ENGINE_SHARED_MEMORY_BRIDGE_CLIENT_NDK_H_
