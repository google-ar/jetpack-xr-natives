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

#include "core/split_engine/android/split_engine_shared_memory_bridge_client_ndk.h"

#include <aidl/imp/split_engine/BnSplitEngineResponseHandler.h>
#include <aidl/imp/split_engine/BnSplitEngineSharedMemoryReverseBridge.h>
#include <aidl/imp/split_engine/ISplitEngineSharedMemoryBridge.h>
#include <android/binder_auto_utils.h>
#include <android/binder_ibinder.h>
#include <android/binder_interface_utils.h>
#include <android/binder_status.h>
#include <android/native_window_aidl.h>
#include <android/native_window_jni.h>
#include <jni.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/common/invocable.h"
#include "core/common/trace.h"
#include "core/split_engine/android/extensions/split_engine_bridge.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

namespace {
using BufferHandle =
    imp::split_engine::SplitEngineSharedMemoryBridgeClient::BufferHandle;
using ISplitEngineSharedMemoryBridge =
    aidl::imp::split_engine::ISplitEngineSharedMemoryBridge;

JNIEnv* GetJNIEnv(JavaVM* java_vm) {
  JNIEnv* env;
  java_vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
  if (!env) {
    IMP_LOG(imp::FATAL)
        << "No JNIEnv available in SplitEngineSharedMemoryBridgeClientNdk";
  }
  return env;
}

absl::Status transformStatus(ndk::ScopedAStatus& status) {
  if (status.isOk()) {
    return absl::OkStatus();
  }
  const char* error_message = status.getMessage();
  switch (status.getServiceSpecificError()) {
    case ISplitEngineSharedMemoryBridge::ERROR_CODE_INVALID_ARGUMENT:
      return absl::InvalidArgumentError(error_message);
    case ISplitEngineSharedMemoryBridge::ERROR_CODE_RESOURCE_ALREADY_EXISTS:
      return absl::AlreadyExistsError(error_message);
    case ISplitEngineSharedMemoryBridge::ERROR_CODE_RESOURCE_NOT_FOUND:
      return absl::NotFoundError(error_message);
    case ISplitEngineSharedMemoryBridge::ERROR_CODE_INTERNAL:
    default:
      return absl::InternalError(error_message);
  }
}

// While SplitEngineSharedMemoryBridge::ProcessRegion is the mechanism for
// sending messages over the bridge from the client to the Service, this class
// is the mechanism for receiving messages from the Service to the client.
class SplitEngineSharedMemoryReverseBridgeHandler
    : public aidl::imp::split_engine::BnSplitEngineSharedMemoryReverseBridge {
 public:
  explicit SplitEngineSharedMemoryReverseBridgeHandler(
      std::function<absl::Status(MessageGroupId)> callback)
      : callback_(callback) {}
  ~SplitEngineSharedMemoryReverseBridgeHandler() = default;

  // TODO: For now we only have one type of message going over the
  // reverse bridge, so it's defined explicitly here. If this expands then we
  // may want to consider defining the reverse bridge's schema in flatbuffers
  // for consistency with the forward bridge schema.
  // TODO: (broken link) - use int64_t instead of int.
  ndk::ScopedAStatus onMessageGroupComplete(int group_id) override {
    IMP_TRACE();

    if (absl::Status release_result = callback_(MessageGroupId(group_id));
        !release_result.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to release message group: "
                 << release_result.ToString();
    }

    return ndk::ScopedAStatus::ok();
  }

 private:
  std::function<absl::Status(MessageGroupId)> callback_;
};

class SplitEngineResponseHandler
    : public aidl::imp::split_engine::BnSplitEngineResponseHandler {
 public:
  SplitEngineResponseHandler() = default;
  ~SplitEngineResponseHandler() = default;

  ndk::ScopedAStatus onResponse(const std::vector<uint8_t>& response) override {
    IMP_TRACE();

    if (callback_) {
      callback_(response);
      callback_ = {};
    }
    return ndk::ScopedAStatus::ok();
  }

  void SetResponseCallback(
      imp::Invocable<void(absl::Span<const uint8_t>)> callback) {
    callback_ = std::move(callback);
  }

 private:
  imp::Invocable<void(absl::Span<const uint8_t>)> callback_;
};

}  // namespace

SplitEngineSharedMemoryBridgeClientNdk::SplitEngineSharedMemoryBridgeClientNdk(
    const ndk::SpAIBinder& bridge_service_handle, JNIEnv* jni_env) {
  bridge_service_ =
      aidl::imp::split_engine::ISplitEngineSharedMemoryBridge::fromBinder(
          bridge_service_handle);
  // Note: the constructor is the only place where we can pass arguments to the
  // concrete SplitEngineSharedMemoryReverseBridgeHandler class, so we use a
  // lambda to pass the message group release callback since it binds later.
  reverse_bridge_ =
      ndk::SharedRefBase::make<SplitEngineSharedMemoryReverseBridgeHandler>(
          [this](MessageGroupId group_id) {
            if (!release_message_group_callback_) {
              return absl::FailedPreconditionError(
                  "Message group callback is not set");
            }
            release_message_group_callback_->OnMessageGroupComplete(group_id);
            return absl::OkStatus();
          });
  jni_env->GetJavaVM(&java_vm_);
  // TODO: Move work out of constructor into static creator, and
  // return an actionable error for the app.
  auto status =
      bridge_service_->initializeBridge(reverse_bridge_, &bridge_handle_);
  if (!status.isOk()) {
    IMP_LOG(imp::FATAL) << "SplitEngineSharedMemoryBridge failed to initialize bridge: "
               << status.getMessage();
  }
}

ClientId SplitEngineSharedMemoryBridgeClientNdk::GetClientId() const {
  IMP_LOG(imp::FATAL) << "GetClientId is unimplemented";
  return 0;
}

MessageGroupId
SplitEngineSharedMemoryBridgeClientNdk::GenerateMessageGroupId() {
  IMP_LOG(imp::FATAL) << "GenerateMessageGroupId is unimplemented";
  return 0;
}

void SplitEngineSharedMemoryBridgeClientNdk::RegisterMessageGroupCallback(
    std::unique_ptr<SplitEngineMessageGroupCallback> callback) {
  // This is used by the SplitEngineSharedMemoryReverseBridgeHandler to release
  // message groups.
  release_message_group_callback_ = std::move(callback);
}

absl::StatusOr<std::unique_ptr<BufferHandle>>
SplitEngineSharedMemoryBridgeClientNdk::RegisterBuffer(
    int fd, size_t buffer_size_bytes) {
  IMP_TRACE();
  ndk::SpAIBinder buffer_handle;
  auto pfd = ndk::ScopedFileDescriptor(dup(fd));
  auto status = bridge_service_->registerBuffer(
      bridge_handle_, pfd, buffer_size_bytes, &buffer_handle);
  if (!status.isOk()) {
    IMP_LOG(imp::ERROR) << "SplitEngineSharedMemoryBridge failed to register buffer, "
               << status.getMessage();
    return transformStatus(status);
  }
  return std::make_unique<NdkBufferHandle>(buffer_handle);
}

absl::Status SplitEngineSharedMemoryBridgeClientNdk::ProcessRegion(
    const BufferHandle& buffer_handle, int offset_bytes,
    int region_length_bytes) {
  IMP_TRACE();
  auto ndk_buffer_handle =
      static_cast<const NdkBufferHandle*>(&buffer_handle)->buffer_handle_;
  ndk::ScopedAStatus status = bridge_service_->processRegion(
      ndk_buffer_handle, offset_bytes, region_length_bytes);
  return transformStatus(status);
}

absl::StatusOr<jobject>
SplitEngineSharedMemoryBridgeClientNdk::CreateExternalTextureSurface(
    const std::vector<TextureId>& in_texture_ids) {
  IMP_TRACE();
  std::vector<int64_t> texture_ids(in_texture_ids.size());
  for (int i = 0; i < in_texture_ids.size(); ++i) {
    texture_ids[i] = static_cast<int64_t>(in_texture_ids[i]);
  }

  aidl::android::hardware::NativeWindow native_window;
  bridge_service_->createExternalTextureSurface(bridge_handle_, texture_ids,
                                                &native_window);

  JNIEnv* env = GetJNIEnv(java_vm_);
  return ANativeWindow_toSurface(env, native_window.get());
}

absl::Status
SplitEngineSharedMemoryBridgeClientNdk::SetExternalTextureSurfaceSize(
    TextureId in_texture_id, int32_t width, int32_t height) {
  IMP_TRACE();
  ndk::ScopedAStatus status = bridge_service_->setExternalTextureSurfaceSize(
      bridge_handle_, static_cast<int64_t>(in_texture_id), width, height);
  if (!status.isOk()) {
    IMP_LOG(imp::ERROR) << "SplitEngineSharedMemoryBridge failed to set external "
                  "texture surface size, "
               << status.getMessage();
    return transformStatus(status);
  }
  return absl::OkStatus();
}

absl::Status SplitEngineSharedMemoryBridgeClientNdk::SendRequest(
    absl::Span<const uint8_t> data,
    imp::Invocable<void(absl::Span<const uint8_t>)> callback) {
  IMP_TRACE();
  std::shared_ptr<SplitEngineResponseHandler> handler =
      ndk::SharedRefBase::make<SplitEngineResponseHandler>();
  handler->SetResponseCallback(std::move(callback));

  // AIDL byte[] converts to std::vector<uint8_t>, so we have to make a copy.
  std::vector<uint8_t> data_vector(data.begin(), data.end());
  auto status =
      bridge_service_->sendRequest(bridge_handle_, data_vector, handler);
  if (!status.isOk()) {
    IMP_LOG(imp::ERROR) << "SplitEngineSharedMemoryBridge failed to send request, "
               << status.getMessage();
    return transformStatus(status);
  }
  return absl::OkStatus();
}

}  // namespace imp::split_engine
