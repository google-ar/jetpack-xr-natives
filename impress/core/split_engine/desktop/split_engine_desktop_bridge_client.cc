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

#include "core/split_engine/desktop/split_engine_desktop_bridge_client.h"

#include <jni.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <thread>  // NOLINT
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "third_party/grpc/include/grpcpp/client_context.h"
#include "third_party/grpc/include/grpcpp/support/client_callback.h"
#include "third_party/grpc/include/grpcpp/support/status.h"
#include "core/common/invocable.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/desktop/split_engine_desktop_bridge.grpc.pb.h"
#include "core/split_engine/desktop/split_engine_desktop_bridge.proto.h"
#include "core/split_engine/desktop/utils/message_group_completion_client_reactor.h"
#include "core/split_engine/desktop/utils/split_engine_desktop_bridge_utils.h"
#include "core/split_engine/message_group_monitor.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

using BufferHandle = SplitEngineSharedMemoryBridgeClient::BufferHandle;

absl::StatusOr<std::unique_ptr<SplitEngineDesktopBridgeClient>>
SplitEngineDesktopBridgeClient::Create(
    std::unique_ptr<SplitEngineDesktopBridge::Stub> stub) {
  std::unique_ptr<SplitEngineDesktopBridgeClient> client =
      absl::WrapUnique(new SplitEngineDesktopBridgeClient(std::move(stub)));
  MP_RETURN_IF_ERROR(client->Initialize());
  return client;
}

SplitEngineDesktopBridgeClient::SplitEngineDesktopBridgeClient(
    std::unique_ptr<SplitEngineDesktopBridge::Stub> stub)
    : client_id_(reinterpret_cast<ClientId>(this)), stub_(std::move(stub)) {}

absl::Status SplitEngineDesktopBridgeClient::Initialize() {
  // TODO: (broken link) - remove this once ClientID is moved to the higher level
  // components.
  MP_RETURN_IF_ERROR(MessageGroupMonitor::ConnectClient(client_id_));

  struct {
    grpc::ClientContext context;
    InitializeBridgeRequest request;
    InitializeBridgeResponse response;
  } initialize_bridge_args;

  absl::Status result;
  absl::Notification notification;
  stub_->async()->InitializeBridge(
      &initialize_bridge_args.context, &initialize_bridge_args.request,
      &initialize_bridge_args.response,
      [this, &notification, &initialize_bridge_args,
       &result](grpc::Status status) {
        absl::Cleanup cleanup = [&notification] { notification.Notify(); };
        result = status;
        if (!result.ok()) {
          IMP_LOG(imp::ERROR) << "Failed to initialize bridge: " << result;
          return;
        }
        IMP_LOG(imp::INFO) << " bridge id: "
                  << initialize_bridge_args.response.bridge_id()
                  << " fd receiver: "
                  << initialize_bridge_args.response.fd_receiver();
        uds_path_ = initialize_bridge_args.response.fd_receiver();
        bridge_id_ = initialize_bridge_args.response.bridge_id();
      });

  notification.WaitForNotification();
  if (!result.ok()) {
    if (const absl::Status status =
            MessageGroupMonitor::DisconnectClient(client_id_);
        !status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to disconnect client: " << status;
    }
    return result;
  }

  message_group_completion_reactor_ =
      std::make_unique<MessageGroupCompletionClientReactor>(
          bridge_id_,
          [this](grpc::ClientContext* context,
                 MessageGroupCompletionRequest* request,
                 grpc::ClientReadReactor<MessageGroupCompletionResponse>*
                     reactor) {
            stub_->async()->ReadMessageGroupCompletions(context, request,
                                                        reactor);
          },
          [this](MessageGroupId message_group_id) {
            
          });

  heartbeat_thread_ =
      std::thread(&SplitEngineDesktopBridgeClient::Heartbeat, this,
                  absl::Milliseconds(
                      initialize_bridge_args.response.heartbeat_interval_ms()));

  return result;
}

SplitEngineDesktopBridgeClient::~SplitEngineDesktopBridgeClient() {
  {
    absl::MutexLock lock(heartbeat_state_mutex_);
    heartbeat_state_ = HeartbeatState::kStop;
  }

  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }

  // Need to make sure that reactor is destructed before the client is removed
  // from the SplitEngineBridgeSender.
  message_group_completion_reactor_.reset();

  
}

void SplitEngineDesktopBridgeClient::Heartbeat(
    const absl::Duration heartbeat_interval) {
  struct HeartbeatRequestArgs {
    grpc::ClientContext context;
    HeartbeatRequest request;
    google::rpc::Status response;
  };

  absl::MutexLock lock(heartbeat_state_mutex_);
  while (!heartbeat_state_mutex_.AwaitWithTimeout(
      absl::Condition(
          // Unary plus in the beginning of the _captureless_ lambda will
          // convert it to a function pointer and absl::Condition will gladly
          // accept it.
          +[/*captureless*/](HeartbeatState* state) -> bool {
            return *state == HeartbeatState::kStop;
          },
          &heartbeat_state_),
      heartbeat_interval)) {
    auto args = std::make_shared<HeartbeatRequestArgs>();
    args->request.set_bridge_id(bridge_id_);
    stub_->async()->Heartbeat(
        &args->context, &args->request, &args->response,
        [args](grpc::Status status) { CHECK_OK(status); });
  }
}

class BufferHandleImpl : public BufferHandle {
 public:
  explicit BufferHandleImpl(BufferId buffer_id) : kBufferId(buffer_id) {}
  const BufferId kBufferId;
};

static absl::Status FromGrpcStatus(const grpc::Status& status) {
  // Static cast is valid, because absl::StatusCode and grpc::StatusCode
  // match.
  return absl::Status(static_cast<absl::StatusCode>(status.error_code()),
                      status.error_message());
}

static absl::Status FromGrpcStatus(const ::google::rpc::Status& status) {
  return absl::Status(static_cast<absl::StatusCode>(status.code()),
                      status.message());
}

// TODO: (broken link) - consider returning Future instead of blocking the thread.
absl::StatusOr<std::unique_ptr<BufferHandle>>
SplitEngineDesktopBridgeClient::RegisterBuffer(int fd,
                                               size_t buffer_size_bytes) {
  // File descriptor has to be properly transferred between two processes.
  {
    FileDescriptorMetadata metadata;
    // FileDescriptorMetadata may have some padding bytes.
    // Zeroing them out explicitly to make MSAN happy.
    memset(&metadata, 0, sizeof(metadata));
    metadata.bridge_id = bridge_id_;
    metadata.fd = fd;
    metadata.size = buffer_size_bytes;

    if (sender_ == nullptr) {
      sender_ = std::make_unique<FileDescriptorSender>(uds_path_);
    }
    MP_RETURN_IF_ERROR(sender_->Send(fd, metadata));
  }

  struct RegisterBufferRequestArgs {
    grpc::ClientContext context;
    RegisterBufferRequest request;
    RegisterBufferResponse response;
  };

  auto args = std::make_shared<RegisterBufferRequestArgs>();
  args->request.set_bridge_id(bridge_id_);
  args->request.set_file_descriptor(fd);
  args->request.set_size_bytes(buffer_size_bytes);
  absl::StatusOr<std::unique_ptr<BufferHandle>> result;
  absl::Notification notification;
  stub_->async()->RegisterBuffer(
      &args->context, &args->request, &args->response,
      [&notification, &result, args](grpc::Status status) {
        if (status.ok()) {
          result =
              std::make_unique<BufferHandleImpl>(args->response.buffer_id());
        } else {
          result = FromGrpcStatus(status);
        }
        notification.Notify();
      });

  notification.WaitForNotification();

  return result;
}

// TODO: (broken link) - consider returning Future instead of blocking the thread.
absl::Status SplitEngineDesktopBridgeClient::ProcessRegion(
    const BufferHandle& buffer_handle, int offset_bytes,
    int region_length_bytes) {
  const BufferId buffer_id =
      static_cast<const BufferHandleImpl&>(buffer_handle).kBufferId;
  struct ProcessRegionRequestArgs {
    grpc::ClientContext context;
    ProcessRegionRequest request;
    ::google::rpc::Status response;
  };

  auto args = std::make_shared<ProcessRegionRequestArgs>();
  args->request.set_bridge_id(bridge_id_);
  args->request.set_buffer_id(buffer_id);
  args->request.set_offset_bytes(offset_bytes);
  args->request.set_region_length_bytes(region_length_bytes);
  absl::Status result;
  absl::Notification notification;
  stub_->async()->ProcessRegion(
      &args->context, &args->request, &args->response,
      [&notification, &result, args](grpc::Status status) {
        if (status.ok()) {
          result = FromGrpcStatus(args->response);
        } else {
          result = FromGrpcStatus(status);
        }
        notification.Notify();
      });

  notification.WaitForNotification();

  return result;
}

absl::Status SplitEngineDesktopBridgeClient::SendRequest(
    absl::Span<const uint8_t> data,
    imp::Invocable<void(absl::Span<const uint8_t>)> callback) {
  struct SendRequestRequestArgs {
    grpc::ClientContext context;
    SendRequestRequest request;
    SendRequestResponse response;
  };

  // Using shared_ptr to guarantee that the args will be destroyed even if the
  // gRPC callback is never called.
  auto args = std::make_shared<SendRequestRequestArgs>();
  args->request.set_bridge_id(bridge_id_);
  args->request.set_data(data.data(), data.size());

  // gRPC callback is std::function and it's copy-only. In turn, lambda is
  // supposed to be copy-only too. imp::Invocable is move-only, so using
  // shared_ptr to capture "copyable" callback in the lambda below and guarantee
  // that it will be destroyed even if the gRPC callback is never called.
  auto callback_ptr =
      std::make_shared<imp::Invocable<void(absl::Span<const uint8_t>)>>(
          std::move(callback));

  stub_->async()->SendRequest(
      &args->context, &args->request, &args->response,
      [args, callback_ptr](grpc::Status status) {
        
        (*callback_ptr)(absl::MakeConstSpan(
            reinterpret_cast<const uint8_t*>(args->response.data().data()),
            args->response.data().size()));
      });
  return absl::OkStatus();
}

absl::StatusOr<jobject>
SplitEngineDesktopBridgeClient::CreateExternalTextureSurface(
    const std::vector<TextureId>& in_texture_ids) {
  return absl::UnimplementedError(
      "External texture surface is not supported on desktop.");
}

absl::Status SplitEngineDesktopBridgeClient::SetExternalTextureSurfaceSize(
    TextureId in_texture_id, int32_t width, int32_t height) {
  return absl::UnimplementedError(
      "External texture surface is not supported on desktop.");
}

}  // namespace imp::split_engine
