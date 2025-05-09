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

#include "core/split_engine/android/split_engine_shared_memory_bridge_service.h"

#include <aidl/imp/split_engine/BnSplitEngineToken.h>
#include <android/binder_auto_utils.h>
#include <android/binder_interface_utils.h>
#include <android/native_window_aidl.h>
#include <android/native_window_jni.h>
#include <jni.h>

#include <cstdint>
#include <memory>
#include <vector>

#include "aidl/imp/split_engine/ISplitEngineResponseHandler.h"
#include "aidl/imp/split_engine/ISplitEngineSharedMemoryBridge.h"
#include "aidl/imp/split_engine/ISplitEngineSharedMemoryReverseBridge.h"
#include "absl/status/status.h"
#include "core/async/executor.h"
#include "core/common/enum_flags.h"
#include "core/common/registry.h"
#include "core/common/trace.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/view/base_view.h"

namespace imp::split_engine {

namespace {

ndk::ScopedAStatus TransformStatus(absl::Status status) {
  if (status.ok()) {
    return ndk::ScopedAStatus::ok();
  }
  auto error_message =
      status.ToString(absl::StatusToStringMode::kWithNoExtraData);
  switch (status.code()) {
    case absl::StatusCode::kInvalidArgument:
      return ndk::ScopedAStatus::fromServiceSpecificErrorWithMessage(
          SplitEngineSharedMemoryBridgeService::ISplitEngineSharedMemoryBridge::
              ERROR_CODE_INVALID_ARGUMENT,
          error_message.c_str());
    case absl::StatusCode::kNotFound:
      return ndk::ScopedAStatus::fromServiceSpecificErrorWithMessage(
          SplitEngineSharedMemoryBridgeService::ISplitEngineSharedMemoryBridge::
              ERROR_CODE_RESOURCE_NOT_FOUND,
          error_message.c_str());
    case absl::StatusCode::kAlreadyExists:
      return ndk::ScopedAStatus::fromServiceSpecificErrorWithMessage(
          SplitEngineSharedMemoryBridgeService::ISplitEngineSharedMemoryBridge::
              ERROR_CODE_RESOURCE_ALREADY_EXISTS,
          error_message.c_str());
    case absl::StatusCode::kInternal:
    default:
      return ndk::ScopedAStatus::fromServiceSpecificErrorWithMessage(
          SplitEngineSharedMemoryBridgeService::ISplitEngineSharedMemoryBridge::
              ERROR_CODE_INTERNAL,
          error_message.c_str());
  }
}
}  // namespace

constexpr int kWaitTimeIntervalForAsyncSurfaceCreationMicroseconds = 10;

SplitEngineSharedMemoryBridgeService::BridgeHandle::BridgeHandle(
    SplitEngineSharedMemoryBridgeService& bridgeService)
    : bridgeService_(bridgeService) {}

SplitEngineSharedMemoryBridgeService::BridgeHandle::~BridgeHandle() {
  bridgeService_.CleanupBridge(bridge_id_);
}

::ndk::SpAIBinder
SplitEngineSharedMemoryBridgeService::BridgeHandle::CreateBridgeBinder(
    SplitEngineSharedMemoryBridgeService& bridgeService) {
  auto token = ndk::SharedRefBase::make<BridgeHandle>(bridgeService);
  auto binder = token->asBinder();
  auto bridge_id = reinterpret_cast<BridgeId>(binder.get());
  token->SetBridgeId(bridge_id);
  return binder;
}

SplitEngineSharedMemoryBridgeService::BufferHandle::BufferHandle(
    SplitEngineSharedMemoryBridgeService& bridgeService)
    : bridgeService_(bridgeService) {}

SplitEngineSharedMemoryBridgeService::BufferHandle::~BufferHandle() {
  bridgeService_.CleanupBuffer(buffer_id_);
}

::ndk::SpAIBinder
SplitEngineSharedMemoryBridgeService::BufferHandle::CreateBufferBinder(
    SplitEngineSharedMemoryBridgeService& bridgeService) {
  auto token = ndk::SharedRefBase::make<BufferHandle>(bridgeService);
  auto binder = token->asBinder();
  auto buffer_id = reinterpret_cast<BufferId>(binder.get());
  token->SetBufferId(buffer_id);
  return binder;
}

SplitEngineSharedMemoryBridgeService::SplitEngineSharedMemoryBridgeService(
    BaseView& view, Executor* executor)
    : view_(&view), foreground_executor_(executor), impl_(view, executor) {}

void SplitEngineSharedMemoryBridgeService::Update(BaseView& view,
                                                  Executor* executor) {
  view_ = &view;
  foreground_executor_ = executor;
  impl_.Update(view, executor);
}

ndk::ScopedAStatus SplitEngineSharedMemoryBridgeService::initializeBridge(
    const std::shared_ptr<
        ::aidl::imp::split_engine::ISplitEngineSharedMemoryReverseBridge>&
        in_reverse_bridge,
    ::ndk::SpAIBinder* out_bridge_handle) {
  IMP_TRACE();

  auto binder = BridgeHandle::CreateBridgeBinder(*this);
  auto bridge_id = reinterpret_cast<BridgeId>(binder.get());
  if (auto status = impl_.InitializeBridge(
          bridge_id,
          [in_reverse_bridge](MessageGroupId message_group_id) {
            in_reverse_bridge->onMessageGroupComplete(message_group_id);
          });
      !status.ok()) {
    return TransformStatus(status);
  }
  // Grant permission to phones apps by default since their nodes would never be
  // attached to a subspace.
  foreground_executor_->Schedule([this, bridge_id]() {
    view_->GetRegistry()
        .Get<imp::split_engine::SplitEngineRenderer>()
        ->get()
        .AddAppPermission(
            bridge_id,
            imp::ToFlags(AppPermissionTypes::kHasUnrestrictedSystemAccess));
  });
  *out_bridge_handle = binder;
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus SplitEngineSharedMemoryBridgeService::registerBuffer(
    const ::ndk::SpAIBinder& in_bridge_handle,
    const ::ndk::ScopedFileDescriptor& in_file_descriptor,
    int32_t in_size_bytes, ::ndk::SpAIBinder* out_buffer_handle) {
  IMP_TRACE();
  int fd = in_file_descriptor.get();

  auto binder = BufferHandle::CreateBufferBinder(*this);
  auto buffer_id = reinterpret_cast<BufferId>(binder.get());

  auto bridge_id = reinterpret_cast<BridgeId>(in_bridge_handle.get());
  if (auto status =
          impl_.RegisterBuffer(bridge_id, buffer_id, fd, in_size_bytes);
      !status.ok()) {
    return TransformStatus(status);
  }
  *out_buffer_handle = binder;
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus SplitEngineSharedMemoryBridgeService::processRegion(
    const ::ndk::SpAIBinder& buffer_handle, int32_t in_offset_bytes,
    int32_t in_region_length_bytes) {
  IMP_TRACE();
  auto buffer_id = reinterpret_cast<BufferId>(buffer_handle.get());
  if (auto status = impl_.ProcessRegion(
          buffer_id, std::make_shared<NdkMessageGroupStorage>(buffer_handle),
          in_offset_bytes, in_region_length_bytes);
      !status.ok()) {
    return TransformStatus(status);
  }

  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus
SplitEngineSharedMemoryBridgeService::createExternalTextureSurface(
    const ::ndk::SpAIBinder& in_bridge_handle,
    const std::vector<int64_t>& in_texture_ids,
    aidl::android::hardware::NativeWindow* out_surface) {
  IMP_TRACE();
  BridgeId bridge_id = reinterpret_cast<BridgeId>(in_bridge_handle.get());
  jobject surface_reference;

  // Convert the long texture ids received over the AIDL boundary to TextureId.
  std::vector<TextureId> texture_ids(in_texture_ids.size());
  for (int i = 0; i < in_texture_ids.size(); ++i) {
    texture_ids[i] = static_cast<TextureId>(in_texture_ids[i]);
  }

  if (auto status = impl_.CreateExternalTextureSurface(bridge_id, texture_ids,
                                                       surface_reference);
      !status.ok()) {
    return TransformStatus(status);
  }

  *out_surface =
      aidl::android::hardware::NativeWindow(ANativeWindow_fromSurface(
          impl_.View().GetContext().GetJniEnv(), surface_reference));
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus
SplitEngineSharedMemoryBridgeService::setExternalTextureSurfaceSize(
    const ::ndk::SpAIBinder& in_bridge_handle, int64_t in_texture_id,
    int32_t in_width, int32_t in_height) {
  BridgeId bridge_id = reinterpret_cast<BridgeId>(in_bridge_handle.get());
  if (auto status = impl_.SetExternalTextureSurfaceSize(
          bridge_id, in_texture_id, in_width, in_height);
      !status.ok()) {
    return TransformStatus(status);
  }
  return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus SplitEngineSharedMemoryBridgeService::sendRequest(
    const ::ndk::SpAIBinder& in_bridge_handle,
    const std::vector<uint8_t>& in_data,
    const std::shared_ptr<
        ::aidl::imp::split_engine::ISplitEngineResponseHandler>& in_handler) {
  BridgeId bridge_id = reinterpret_cast<uintptr_t>(in_bridge_handle.get());
  if (auto status = impl_.SendRequest(
          bridge_id, in_data,
          [handler = in_handler](const std::vector<uint8_t>& response) {
            handler->onResponse(response);
          });
      !status.ok()) {
    return TransformStatus(status);
  }
  return ndk::ScopedAStatus::ok();
}

void SplitEngineSharedMemoryBridgeService::CleanupBridge(BridgeId bridge_id) {
  impl_.CleanupBridge(bridge_id);
}

void SplitEngineSharedMemoryBridgeService::CleanupBuffer(BufferId buffer_id) {
  impl_.CleanupBuffer(buffer_id);
}

}  // namespace imp::split_engine
