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

#include "core/split_engine/desktop/split_engine_desktop_bridge_service_impl.h"

#include <cstdint>
#include <memory>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/strings/str_format.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "third_party/grpc/include/grpcpp/server_context.h"
#include "third_party/grpc/include/grpcpp/support/server_callback.h"
#include "third_party/grpc/include/grpcpp/support/status.h"
#include "core/async/executor.h"
#include "core/common/enum_flags.h"
#include "core/common/registry.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_service_impl.h"
#include "core/split_engine/desktop/utils/message_group_completion_server_reactor.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_utils.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/view/base_view.h"

namespace imp::split_engine {

SplitEngineDesktopBridgeServiceImpl::SplitEngineDesktopBridgeServiceImpl(
    BaseView& view, Executor& executor)
    : view_(view),
      foreground_executor_(executor),
      impl_(view, &executor),
      heartbeat_monitor_(
          kPulseCheckInterval, kTTL,
          [this](BridgeId bridge_id) {
            impl_.CleanupBridge(bridge_id);
            absl::MutexLock lock(bridge_data_mutex_);
            auto it = bridge_data_.find(bridge_id);
            if (it == bridge_data_.end()) {
              IMP_LOG(imp::WARNING) << "Bridge " << bridge_id << " not found.";
              return;
            }
            static_cast<MessageGroupCompletionServerReactor*>(
                it->second.reactor)
                ->Shutdown(grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                        "Bridge is dead."));
            bridge_data_.erase(it);
          }),
      uds_path_(absl::StrFormat("/tmp/impress-uds-%d-XXXXXXXXX", getpid())) {
  mktemp(const_cast<char*>(uds_path_.c_str()));
  fd_receiver_ = std::make_unique<FileDescriptorReceiver>(
      uds_path_, [this](int fd, const FileDescriptorMetadata& metadata) {
        IMP_LOG(imp::INFO) << "Received file descriptor: " << fd << " " << metadata.size;
        absl::MutexLock lock(fd_metadata_mutex_);
        fd_metadata_map_[metadata.bridge_id][metadata.fd] = {metadata.bridge_id,
                                                             fd, metadata.size};
      });

  
}

grpc::ServerUnaryReactor* SplitEngineDesktopBridgeServiceImpl::InitializeBridge(
    grpc::CallbackServerContext* context,
    const InitializeBridgeRequest* request,
    InitializeBridgeResponse* response) {
  grpc::ServerUnaryReactor* reactor = context->DefaultReactor();

  const BridgeId bridge_id = GenerateBridgeId();
  {
    absl::MutexLock lock(bridge_data_mutex_);
    
    bridge_data_.emplace(bridge_id, nullptr);
  }

  

  if (auto status = impl_.InitializeBridge(
          bridge_id,
          [bridge_id, this](MessageGroupId message_group_id) {
            absl::MutexLock lock(bridge_data_mutex_);
            auto it = bridge_data_.find(bridge_id);
            if (it == bridge_data_.end()) {
              IMP_LOG(imp::WARNING) << "Bridge " << bridge_id << " not found.";
              return;
            }
            
            static_cast<MessageGroupCompletionServerReactor*>(
                it->second.reactor)
                ->ReportCompletion(message_group_id);
          });
      !status.ok()) {
    reactor->Finish(status);
    return reactor;
  }

  // Mimic behavior of the Android bridge and grant permission to apps by
  // default.
  foreground_executor_.Schedule([this, bridge_id]() {
    view_.GetRegistry()
        .Get<imp::split_engine::SplitEngineRenderer>()
        ->get()
        .AddAppPermission(
            bridge_id,
            imp::ToFlags(AppPermissionTypes::kHasUnrestrictedSystemAccess));
  });

  response->set_heartbeat_interval_ms(
      absl::ToInt64Milliseconds(kHeartbeatInterval));
  response->set_bridge_id(bridge_id);
  response->set_fd_receiver(uds_path_);
  reactor->Finish(grpc::Status::OK);
  return reactor;
}

grpc::ServerUnaryReactor* SplitEngineDesktopBridgeServiceImpl::Heartbeat(
    grpc::CallbackServerContext* context, const HeartbeatRequest* request,
    google::rpc::Status* response) {
  // TODO: (broken link) - can _this_ client operate on the request->bridge_id()?
  IMP_LOG(imp::ERROR) << "Heartbeat " << request->bridge_id();
  grpc::ServerUnaryReactor* reactor = context->DefaultReactor();
  const absl::Status status =
      heartbeat_monitor_.Heartbeat(request->bridge_id());
  response->set_code(static_cast<int>(status.code()));
  response->set_message(status.message());
  reactor->Finish(status);
  return reactor;
}

grpc::ServerUnaryReactor* SplitEngineDesktopBridgeServiceImpl::RegisterBuffer(
    grpc::CallbackServerContext* context, const RegisterBufferRequest* request,
    RegisterBufferResponse* response) {
  // TODO: (broken link) - can _this_ client operate on the request->bridge_id()?
  grpc::ServerUnaryReactor* reactor = context->DefaultReactor();
  FileDescriptorMetadata metadata;
  {
    absl::MutexLock lock(fd_metadata_mutex_);
    auto bridge_it = fd_metadata_map_.find(request->bridge_id());
    if (bridge_it == fd_metadata_map_.end()) {
      reactor->Finish(grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                   "Bridge not found."));
      return reactor;
    }

    auto it = bridge_it->second.find(request->file_descriptor());
    if (it == bridge_it->second.end()) {
      reactor->Finish(grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                   "File descriptor not found."));
      return reactor;
    }

    if (it->second.size != request->size_bytes()) {
      reactor->Finish(grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                   "File descriptor size mismatch."));
      return reactor;
    }

    if (it->second.bridge_id != request->bridge_id()) {
      reactor->Finish(grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                   "File descriptor bridge id mismatch."));
      return reactor;
    }

    metadata = it->second;
  }

  const BufferId buffer_id = GetNewBufferId();

  if (auto status = impl_.RegisterBuffer(request->bridge_id(), buffer_id,
                                         metadata.fd, metadata.size);
      !status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to register buffer: " << status;
    reactor->Finish(status);
    return reactor;
  }

  response->set_buffer_id(buffer_id);
  reactor->Finish(grpc::Status::OK);
  return reactor;
}

grpc::ServerUnaryReactor* SplitEngineDesktopBridgeServiceImpl::ProcessRegion(
    grpc::CallbackServerContext* context, const ProcessRegionRequest* request,
    google::rpc::Status* response) {
  // TODO: (broken link) - can _this_ client operate on the request->bridge_id()?
  grpc::ServerUnaryReactor* reactor = context->DefaultReactor();
  if (auto status = impl_.ProcessRegion(
          request->buffer_id(),
          std::make_shared<
              SplitEngineSharedMemoryBridgeServiceImpl::MessageGroupStorage>(),
          request->offset_bytes(), request->region_length_bytes());
      !status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to process region: " << status;
    reactor->Finish(status);
    return reactor;
  }
  reactor->Finish(grpc::Status::OK);
  return reactor;
}

grpc::ServerUnaryReactor* SplitEngineDesktopBridgeServiceImpl::SendRequest(
    grpc::CallbackServerContext* context, const SendRequestRequest* request,
    SendRequestResponse* response) {
  // TODO: (broken link) - can _this_ client operate on the request->bridge_id()?

  grpc::ServerUnaryReactor* reactor = context->DefaultReactor();

  if (auto status = impl_.SendRequest(
          request->bridge_id(),
          // TODO: (broken link) - update the API to take a span to avoid the copy
          // here.
          std::vector<uint8_t>(request->data().begin(), request->data().end()),
          [response, reactor](const std::vector<uint8_t>& data) {
            response->set_data(data.data(), data.size());
            reactor->Finish(grpc::Status::OK);
          });
      !status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to send request: " << status;
    reactor->Finish(status);
    return reactor;
  }

  return reactor;
}

grpc::ServerWriteReactor<MessageGroupCompletionResponse>*
SplitEngineDesktopBridgeServiceImpl::ReadMessageGroupCompletions(
    grpc::CallbackServerContext* context,
    const MessageGroupCompletionRequest* request) {
  const BridgeId bridge_id = request->bridge_id();

  absl::MutexLock lock(bridge_data_mutex_);

  auto it = bridge_data_.find(bridge_id);
  if (it == bridge_data_.end()) {
    auto reactor = new MessageGroupCompletionServerReactor();
    reactor->Shutdown(grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                   "Bridge not found."));

    return reactor;
  }

  if (it->second.reactor) {
    auto reactor = new MessageGroupCompletionServerReactor();
    reactor->Shutdown(grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                   "Already registered."));

    return reactor;
  }

  // gRPC takes ownership of the reactor, so we don't need to worry about
  // deleting it.
  it->second.reactor = new MessageGroupCompletionServerReactor();

  return it->second.reactor;
}

}  // namespace imp::split_engine
