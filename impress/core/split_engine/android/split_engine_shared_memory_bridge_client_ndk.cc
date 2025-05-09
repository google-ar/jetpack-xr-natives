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
#include <android/native_window_aidl.h>
#include <android/native_window_jni.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "core/common/log.h"
#include "core/common/trace.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"

namespace imp::split_engine {

using WorkScheduler = SplitEngineSharedMemoryBridgeClient::WorkScheduler;

namespace {
using BufferHandle =
    imp::split_engine::SplitEngineSharedMemoryBridgeClient::BufferHandle;
using Result = imp::split_engine::SplitEngineSharedMemoryBridgeClient::Result;
using ISplitEngineSharedMemoryBridge =
    aidl::imp::split_engine::ISplitEngineSharedMemoryBridge;

Result transformStatus(ndk::ScopedAStatus status) {
  if (status.isOk()) {
    return Result(true);
  }
  return Result(false, status.getMessage());
}

JNIEnv* GetJNIEnv(JavaVM* java_vm) {
  JNIEnv* env;
  java_vm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6);
  if (!env) {
    IMP_LOG(imp::FATAL)
        << "No JNIEnv available in SplitEngineSharedMemoryBridgeClientNdk";
  }
  return env;
}

// While SplitEngineSharedMemoryBridge::ProcessRegion is the mechanism for
// sending messages over the bridge from the client to the Service, this class
// is the mechanism for receiving messages from the Service to the client.
class SplitEngineSharedMemoryReverseBridgeHandler
    : public aidl::imp::split_engine::BnSplitEngineSharedMemoryReverseBridge {
 public:
  SplitEngineSharedMemoryReverseBridgeHandler() = default;
  ~SplitEngineSharedMemoryReverseBridgeHandler() = default;

  // TODO: For now we only have one type of message going over the
  // reverse bridge, so it's defined explicitly here. If this expands then we
  // may want to consider defining the reverse bridge's schema in flatbuffers
  // for consistency with the forward bridge schema.
  ndk::ScopedAStatus onMessageGroupComplete(int group_id) override {
    // This is invoked on the binder thread, so need to delegate work to the
    // executor to avoid concurrent access to state.
    IMP_TRACE();

    if (work_scheduler_ == nullptr) {
      IMP_LOG(imp::FATAL)
          << "Work Scheduler is not available for onMessageGroupComplete.";
    }

    work_scheduler_([this, group_id]() {
      for (const auto& callback : on_message_group_complete_callbacks_) {
        callback(group_id);
      }
    });
    return ndk::ScopedAStatus::ok();
  }

  void AddOnMessageGroupCompleteCallback(
      std::function<void(int)> on_message_group_complete_callback) {
    on_message_group_complete_callbacks_.push_back(
        std::move(on_message_group_complete_callback));
  }

  void SetWorkScheduler(WorkScheduler work_scheduler) {
    // Need to set the work scheduler explicitly, as the
    // OnMessageGroupComplete callback happens on the binder thread, which can't
    // use the impress executor through threadlocal accessors.
    work_scheduler_ = work_scheduler;
  }

 private:
  std::vector<std::function<void(int)>> on_message_group_complete_callbacks_;
  WorkScheduler work_scheduler_;
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
      callback_ = nullptr;
    }
    return ndk::ScopedAStatus::ok();
  }

  void SetResponseCallback(
      std::function<void(const std::vector<uint8_t>&)> callback) {
    callback_ = std::move(callback);
  }

 private:
  std::function<void(const std::vector<uint8_t>&)> callback_;
};

}  // namespace

SplitEngineSharedMemoryBridgeClientNdk::SplitEngineSharedMemoryBridgeClientNdk(
    const ndk::SpAIBinder& bridge_service_handle, JNIEnv* jni_env) {
  bridge_service_ =
      aidl::imp::split_engine::ISplitEngineSharedMemoryBridge::fromBinder(
          bridge_service_handle);
  reverse_bridge_ =
      ndk::SharedRefBase::make<SplitEngineSharedMemoryReverseBridgeHandler>();
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

void SplitEngineSharedMemoryBridgeClientNdk::Initialize(
    WorkScheduler work_scheduler) {
  static_cast<SplitEngineSharedMemoryReverseBridgeHandler*>(
      reverse_bridge_.get())
      ->SetWorkScheduler(work_scheduler);
}

std::unique_ptr<BufferHandle>
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
    return nullptr;
  }
  return std::make_unique<NdkBufferHandle>(buffer_handle);
}

Result SplitEngineSharedMemoryBridgeClientNdk::ProcessRegion(
    const BufferHandle& buffer_handle, size_t offset_bytes,
    size_t region_length_bytes) {
  IMP_TRACE();
  auto ndk_buffer_handle =
      static_cast<const NdkBufferHandle*>(&buffer_handle)->buffer_handle_;
  return transformStatus(bridge_service_->processRegion(
      ndk_buffer_handle, offset_bytes, region_length_bytes));
}

void SplitEngineSharedMemoryBridgeClientNdk::
    RegisterReverseBridgeMessageHandler(std::function<void(int)> handler) {
  static_cast<SplitEngineSharedMemoryReverseBridgeHandler*>(
      reverse_bridge_.get())
      ->AddOnMessageGroupCompleteCallback(std::move(handler));
}

jobject SplitEngineSharedMemoryBridgeClientNdk::CreateExternalTextureSurface(
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

Result SplitEngineSharedMemoryBridgeClientNdk::SetExternalTextureSurfaceSize(
    TextureId in_texture_id, int32_t width, int32_t height) {
  IMP_TRACE();
  return transformStatus(bridge_service_->setExternalTextureSurfaceSize(
      bridge_handle_, static_cast<int64_t>(in_texture_id), width, height));
}

Result SplitEngineSharedMemoryBridgeClientNdk::SendRequest(
    const std::vector<uint8_t>& data,
    std::function<void(const std::vector<uint8_t>&)> callback) {
  IMP_TRACE();
  std::shared_ptr<SplitEngineResponseHandler> handler =
      ndk::SharedRefBase::make<SplitEngineResponseHandler>();
  handler->SetResponseCallback(std::move(callback));
  return transformStatus(
      bridge_service_->sendRequest(bridge_handle_, data, handler));
}

}  // namespace imp::split_engine
