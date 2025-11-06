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

#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_service.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "third_party/grpc/include/grpcpp/server_context.h"
#include "third_party/grpc/include/grpcpp/support/server_callback.h"
#include "third_party/grpc/include/grpcpp/support/status.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_service_impl_facade.h"
#include "core/split_engine/desktop/utils/message_group_completion_server_reactor.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

SplitEngineMMDesktopBridgeService::SplitEngineMMDesktopBridgeService(
    std::unique_ptr<SplitEngineMMDesktopBridgeServiceImplFacade> impl)
    : impl_(std::move(impl)),
      heartbeat_monitor_(kPulseCheckInterval, kTTL, [this](BridgeId bridge_id) {
        IMP_LOG(imp::INFO) << "Bridge " << bridge_id << " is dead.";
        DestroyBridge(bridge_id);
      }) {}

grpc::ServerUnaryReactor* SplitEngineMMDesktopBridgeService::Connect(
    grpc::CallbackServerContext* context, const ConnectRequest* request,
    ConnectResponse* response) {
  grpc::ServerUnaryReactor* reactor = context->DefaultReactor();
  // TODO: (broken link) - limit the number of bridges
  const BridgeId bridge_id = GetNewBridgeId();
  {
    absl::MutexLock lock(&bridge_data_mutex_);
    
    if (bridge_data_.contains(bridge_id)) {
      reactor->Finish(
          grpc::Status(grpc::StatusCode::INTERNAL, "Bridge already exists."));
      return reactor;
    }
    bridge_data_.emplace(bridge_id, nullptr);
  }

  
        auto it = bridge_data_.find(bridge_id);
        if (it == bridge_data_.end()) {
          IMP_LOG(imp::WARNING) << "Bridge " << bridge_id << " not found.";
          return;
        }
        
        static_cast<MessageGroupCompletionServerReactor*>(it->second.reactor)
            ->ReportCompletion(group_id);
      }));

  
  response->set_bridge_id(bridge_id);
  response->set_heartbeat_interval_ms(
      absl::ToInt64Milliseconds(kHeartbeatInterval));
  reactor->Finish(grpc::Status::OK);
  return reactor;
}

grpc::ServerUnaryReactor* SplitEngineMMDesktopBridgeService::Heartbeat(
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

grpc::ServerUnaryReactor* SplitEngineMMDesktopBridgeService::SendRequest(
    grpc::CallbackServerContext* context, const SendRequestRequest* request,
    SendRequestResponse* response) {
  absl::Notification notification;
  grpc::ServerUnaryReactor* reactor = context->DefaultReactor();
  {
    absl::MutexLock lock(&bridge_data_mutex_);
    auto it = bridge_data_.find(request->bridge_id());
    if (it == bridge_data_.end()) {
      reactor->Finish(grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                   "Bridge not found."));
      return reactor;
    }
  }

  if (const auto status = impl_->SendRequest(
          request->bridge_id(),
          absl::MakeSpan(
              reinterpret_cast<const uint8_t*>(request->data().data()),
              request->data().size()),
          [&response, &notification](const std::vector<uint8_t>& data) {
            response->set_data(data.data(), data.size());
            notification.Notify();
          });
      !status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to send request: " << status;
    reactor->Finish(status);
    return reactor;
  }

  notification.WaitForNotification();
  reactor->Finish(grpc::Status::OK);

  return reactor;
}

namespace {
class MessageGroupReaderReactor final
    : public grpc::ServerReadReactor<SendMessageGroupRequest> {
 public:
  MessageGroupReaderReactor(SplitEngineMMDesktopBridgeServiceImplFacade& impl)
      : impl_(impl) {
    StartRead(&request_);
  }
  MessageGroupReaderReactor(MessageGroupReaderReactor&&) = delete;
  MessageGroupReaderReactor& operator=(MessageGroupReaderReactor&&) = delete;
  MessageGroupReaderReactor(const MessageGroupReaderReactor&) = delete;
  MessageGroupReaderReactor& operator=(const MessageGroupReaderReactor&) =
      delete;

  ~MessageGroupReaderReactor() override = default;

  void OnReadDone(bool ok) override {
    if (!ok) {
      if (!bridge_id_.has_value()) {
        Finish(grpc::Status::OK);
      } else {
        Finish(impl_.CloseMessageGroup(bridge_id_.value(), group_id_.value()));
      }
      return;
    }

    if (!bridge_id_.has_value()) {
      bridge_id_ = request_.bridge_id();
      group_id_ = request_.group_id();
      max_group_size_bytes_ = request_.group_max_size_bytes();
    } else if (bridge_id_ != request_.bridge_id() ||
               group_id_ != request_.group_id() ||
               max_group_size_bytes_ != request_.group_max_size_bytes()) {
      Finish(grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                          "Invariants mismatch."));
      return;
    }

    if (const absl::Status status = impl_.SendMessageGroupPart(
            request_.bridge_id(), request_.group_id(),
            request_.group_max_size_bytes(),
            absl::MakeSpan(reinterpret_cast<const uint8_t*>(
                               request_.partial_data().data()),
                           request_.partial_data().size()));
        !status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to send message group part: " << status;
      Finish(status);
      return;
    }

    request_.Clear();
    StartRead(&request_);
  }

  void OnDone() override {}

 private:
  SplitEngineMMDesktopBridgeServiceImplFacade& impl_;
  SendMessageGroupRequest request_;
  std::optional<BridgeId> bridge_id_;
  std::optional<MessageGroupId> group_id_;
  std::optional<size_t> max_group_size_bytes_;
};
}  // namespace

grpc::ServerReadReactor<SendMessageGroupRequest>*
SplitEngineMMDesktopBridgeService::SendMessageGroup(
    grpc::CallbackServerContext*, SendMessageGroupResponse*) {
  return new MessageGroupReaderReactor(*impl_);
}

grpc::ServerWriteReactor<MessageGroupCompletionResponse>*
SplitEngineMMDesktopBridgeService::ReadMessageGroupCompletions(
    grpc::CallbackServerContext* context,
    const MessageGroupCompletionRequest* request) {
  absl::MutexLock lock(&bridge_data_mutex_);
  auto it = bridge_data_.find(request->bridge_id());
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

  it->second.reactor = new MessageGroupCompletionServerReactor();

  return it->second.reactor;
}

void SplitEngineMMDesktopBridgeService::DestroyBridge(BridgeId bridge_id) {
  if (const absl::Status status = impl_->DestroyBridge(bridge_id);
      !status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to destroy bridge: " << status;
  }
  absl::MutexLock lock(&bridge_data_mutex_);
  auto it = bridge_data_.find(bridge_id);
  if (it == bridge_data_.end()) {
    IMP_LOG(imp::ERROR) << "Bridge " << bridge_id << " not found.";
  } else {
    static_cast<MessageGroupCompletionServerReactor*>(it->second.reactor)
        ->Shutdown(grpc::Status(grpc::StatusCode::FAILED_PRECONDITION,
                                "Bridge is dead."));
    bridge_data_.erase(it);
  }
}

BridgeId SplitEngineMMDesktopBridgeService::GetNewBridgeId() {
  absl::MutexLock lock(&bridge_id_mutex_);
  return new_bridge_id++;
}

}  // namespace imp::split_engine
