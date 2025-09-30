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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_VIEW_EXTENSIONS_SPLITENGINEBRIDGE_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_VIEW_EXTENSIONS_SPLITENGINEBRIDGE_H_
#include <jni.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <type_traits>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/jni_helpers.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_bridge_sender.h"

namespace imp::split_engine {

class SplitEngineRequestCallback : public JavaWrapper {
 public:
  explicit SplitEngineRequestCallback(
      JNIEnv* env, std::function<void(const std::vector<uint8_t>&)> callback)
      : JavaWrapper(env,
                    "com/google/imp/splitengine/extensions/RequestCallback",
                    "(J)V", reinterpret_cast<int64_t>(this)) {
    native_on_result_ = callback;
  }

  void OnResult(const std::vector<uint8_t>& response) {
    native_on_result_(response);
  }

  std::function<void(const std::vector<uint8_t>&)> GetNativeCallback() {
    return native_on_result_;
  }

 private:
  std::function<void(const std::vector<uint8_t>&)> native_on_result_;
};

// A wrapper around the Java MessageGroupCallback type. This is used by the
// bridge to release message groups when they are done processing.
class SplitEngineMessageGroupCallback : public JavaWrapper {
 public:
  // This constructor is used by the SplitEngineBridge to create the message
  // group callback initially. The bridge then passes this to the Java side
  // to IRendererConnection.addMessageGroupCallback.
  SplitEngineMessageGroupCallback(JNIEnv* env, BridgeId bridge_id)
      : JavaWrapper(
            env, "com/google/imp/splitengine/extensions/MessageGroupCallback",
            "(J)V", static_cast<int64_t>(bridge_id)),
        // TODO: (broken link) - use long (J) instead of int (I).
        on_message_group_complete_(
            JavaWrapper::GetMethodHandle("onMessageGroupComplete", "(I)V")) {}

  // This constructor is used inside the non-AXR bridge when it receives the
  // message group callback from the java renderer connection to wrap the
  // existing callback.
  explicit SplitEngineMessageGroupCallback(JNIEnv* env,
                                           jobject message_group_callback)
      : JavaWrapper(
            env, message_group_callback,
            "com/google/imp/splitengine/extensions/MessageGroupCallback"),
        // TODO: (broken link) - use long (J) instead of int (I).
        on_message_group_complete_(
            JavaWrapper::GetMethodHandle("onMessageGroupComplete", "(I)V")) {}

  void OnMessageGroupComplete(MessageGroupId message_group_id) {
    JavaWrapper::CallVoidMethod(on_message_group_complete_, message_group_id);
  }

 private:
  const JniHandle on_message_group_complete_;
};

/*
 * SplitEngineBridge is a wrapper around the Java SplitEngineBridge. It is
 * used to register buffers and process regions of the buffers.
 */
class SplitEngineBridge : public JavaWrapper,
                          public SplitEngineSharedMemoryBridgeClient {
  static_assert(std::is_convertible_v<TextureId, jlong>,
                "TextureId must be convertible to jlong");

 public:
  SplitEngineBridge(JNIEnv* env, jobject split_engine_bridge)
      : JavaWrapper(
            env, split_engine_bridge,
            "com/google/imp/splitengine/extensions/IRendererConnection"),
        register_buffer_(JavaWrapper::GetMethodHandle(
            "registerBuffer",
            "(II)Lcom/google/imp/splitengine/extensions/IBufferHandle;")),
        process_region_(JavaWrapper::GetMethodHandle(
            "processRegion",
            "(Lcom/google/imp/splitengine/extensions/IBufferHandle;II)V")),
        create_external_texture_surface_(JavaWrapper::GetMethodHandle(
            "createExternalTextureSurface", "([J)Landroid/view/Surface;")),
        set_external_texture_surface_size_(JavaWrapper::GetMethodHandle(
            "setExternalTextureSurfaceSize", "(JII)V")),
        send_request_(JavaWrapper::GetMethodHandle(
            "sendRequest",
            "([BLcom/google/imp/splitengine/extensions/RequestCallback;)V")),
        close_(JavaWrapper::GetMethodHandle("close", "()V")),
        client_id_(reinterpret_cast<int64_t>(this)) {
    if (absl::Status connect_result =
            SplitEngineBridgeSender::ConnectClient(client_id_);
        !connect_result.ok()) {
      IMP_LOG(imp::FATAL) << "Failed to connect bridge: " << connect_result.ToString();
    }

    // Ensure the message group callback is set up.
    message_group_callback_ =
        std::make_unique<SplitEngineMessageGroupCallback>(env, client_id_);
    JniHandle add_message_group_callback =
        JavaWrapper::GetMethodHandle("addMessageGroupCallback",
                                     "(Lcom/google/imp/splitengine/extensions/"
                                     "MessageGroupCallback;)V");
    JavaWrapper::CallVoidMethod(add_message_group_callback,
                                message_group_callback_->Reference());
  }

  ~SplitEngineBridge() {
    if (absl::Status disconnect_result =
            SplitEngineBridgeSender::DisconnectClient(client_id_);
        !disconnect_result.ok()) {
      IMP_LOG(imp::FATAL) << "Failed to disconnect bridge: "
                 << disconnect_result.ToString();
    }

    JavaWrapper::CallVoidMethod(close_);
  }

  ClientId GetClientId() const override { return client_id_; }

  // TODO: (broken link) - when we can use memory addresses for message groups,
  // this will be removed and the message group id will be the memory address.
  MessageGroupId GenerateMessageGroupId() override {
    return MessageGroupId(++next_message_group_id_);
  }

  // Registers a shared memory buffer file descriptor with the bridge and
  // returns a handle to the shared memory buffer.
  absl::StatusOr<std::unique_ptr<BufferHandle>> RegisterBuffer(
      int fd, size_t buffer_size_bytes) override;

  // Processes a region of a registered buffer specified by offset and bytes.
  absl::Status ProcessRegion(const BufferHandle& buffer_handle,
                             int offset_bytes,
                             int region_length_bytes) override;

  // Creates a texture surface bound to the given external texture id.
  absl::StatusOr<jobject> CreateExternalTextureSurface(
      const std::vector<TextureId>& in_texture_ids) override;

  // Sets the size of an external texture surface bound to the given texture
  // id.
  absl::Status SetExternalTextureSurfaceSize(TextureId in_texture_id,
                                             int32_t width,
                                             int32_t height) override;

  // Sends a flatbuffer request to the backend with a handler for a flatbuffer
  // response.
  absl::Status SendRequest(
      const std::vector<uint8_t>& data,
      std::function<void(const std::vector<uint8_t>&)> callback) override;

 private:
  const JniHandle get_bridge_id_;
  const JniHandle add_message_group_callback_;
  const JniHandle register_buffer_;
  const JniHandle process_region_;
  const JniHandle create_external_texture_surface_;
  const JniHandle set_external_texture_surface_size_;
  const JniHandle send_request_;
  const JniHandle close_;
  const ClientId client_id_;
  std::unique_ptr<SplitEngineMessageGroupCallback> message_group_callback_;
  int32_t next_message_group_id_ = 0;
};
}  // namespace imp::split_engine
#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_ANDROID_VIEW_EXTENSIONS_SPLITENGINEBRIDGE_H_
