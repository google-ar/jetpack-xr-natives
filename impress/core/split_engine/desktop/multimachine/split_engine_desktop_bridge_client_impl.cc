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

#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_client_impl.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <queue>
#include <thread>  // NOLINT
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "third_party/grpc/include/grpcpp/client_context.h"
#include "third_party/grpc/include/grpcpp/support/client_callback.h"
#include "third_party/grpc/include/grpcpp/support/status.h"
#include "core/common/invocable.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge.grpc.pb.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_client.h"
#include "core/split_engine/desktop/utils/message_group_completion_client_reactor.h"
#include "core/split_engine/message_group_monitor.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

namespace {
class SendMessageGroupReactor final
    : public grpc::ClientWriteReactor<SendMessageGroupRequest> {
  enum class ReactorStatus : uint8_t {
    kIdle,         // Initial state.
    kStarted,      // Started the call, the only state that can initiate writes.
    kWriting,      // Writing the next message.
    kTerminating,  // Terminating the call.
    kDead,         // Reactor is dead and will not work anymore.
  };

  using MessageGroupSentCallback =
      SplitEngineMMDesktopBridgeClient::MessageGroupSentCallback;

 public:
  SendMessageGroupReactor(
      SplitEngineMMDesktopBridge::Stub& stub, BridgeId bridge_id,
      MessageGroupId message_group_id, size_t message_group_max_size_bytes,
      MessageGroupSentCallback&& on_message_group_sent_callback)
      : stub_(stub),
        bridge_id_(bridge_id),
        message_group_id_(message_group_id),
        message_group_max_size_bytes_(message_group_max_size_bytes),
        on_message_group_sent_callback_(
            std::move(on_message_group_sent_callback)) {}

  void Start() {
    {
      absl::MutexLock lock(status_mutex_);
      
      status_ = ReactorStatus::kStarted;

      // Initiating RPC call under `status_mutex_` to prevent the race with
      // a `Write` call from `Finish`.
      stub_.async()->SendMessageGroup(&context_, &response_, this);
      StartCall();
    }

    Write();
  }

  // Adds a message to be sent to the server.
  //
  void AddMessage(absl::Span<const uint8_t> message) {
    
    {
      absl::MutexLock lock(messages_mutex_);
      size_t remaining_bytes = message.size();
      // gRPC has 4MB limit per message (i.e. per one "write").
      // Code below chunks the message into multiple pieces with each piece
      // being no larger than 1MB. Chunk size can be finetuned for better
      // performance, but it cannot exceed gRPC limit.
      constexpr size_t chunk_size = 1024 * 1024;
      const uint8_t* data_ptr = message.data();
      while (remaining_bytes > 0) {
        SendMessageGroupRequest request;
        request.set_bridge_id(bridge_id_);
        request.set_group_id(message_group_id_);
        request.set_group_max_size_bytes(message_group_max_size_bytes_);

        // `set_partial_data` will **copy** the data to the request.
        request.set_partial_data(data_ptr,
                                 std::min(remaining_bytes, chunk_size));

        // Prevent unnecessary copy of the data buffer via move.
        messages_.push(std::move(request));

        if (remaining_bytes < chunk_size) {
          // We just added the last chunk, so we can exit the loop.
          break;
        }

        // Adjust the remaining bytes and data pointer.
        remaining_bytes -= chunk_size;
        data_ptr += chunk_size;
      }
    }

    // Initiate the write if needed.
    Write();
  }

  // Schedules the finish of the RPC call.
  //
  // If there are no more messages to send, the RPC call will be finalized
  // immediately. Otherwise, the call will be finalized when the last message
  // is sent.
  void Finish() {
    

    // Mark that we don't expect any more messages.
    SetNoMoreMessages();

    {
      absl::MutexLock lock(status_mutex_);
      if (status_ == ReactorStatus::kDead ||
          status_ == ReactorStatus::kTerminating) {
        return;
      }
    }

    Write();
  }

  void Await() { done_.WaitForNotification(); }

  // Callback for client-side event.
  void OnWriteDone(bool ok) override {
    if (!ok) {
      // OnDone will be called
      return;
    }

    {
      absl::MutexLock lock(status_mutex_);
      
      status_ = ReactorStatus::kStarted;

      {
        absl::MutexLock lock(messages_mutex_);
        
        messages_.pop();
      }
    }

    // Attempt to write the next message if any.
    Write();
  }

  // Callback for server-side event, can be called in parallel to `OnWriteDone`.
  void OnDone(const grpc::Status& status) override {
    // Server completed the RPC call.
    
    {
      absl::MutexLock lock(status_mutex_);
      status_ = ReactorStatus::kDead;
    }

    {
      absl::MutexLock lock(messages_mutex_);
      
    }

    // Message group was delivered to the server successfully.
    done_.Notify();
  }

  void Write() {
    // StartWrite, StartWritesDone should be called outside of the lock, because
    // OnWriteDone can be called from them.
    enum class Action : uint8_t {
      kNone,
      kStartWrite,
      kStartWritesDone,
    };

    SendMessageGroupRequest* next_message = nullptr;

    Action action = Action::kNone;

    // We want lambda to be destroyed prior to gRPC calls.
    // StartWritesDone can almost immediately trigger OnDone in different thread
    // (e.g. in case of inprocess channels). OnDone will allow destruction of
    // `this`. In turn there will be race between destruction of `this` and
    // completion of this method.
    {
      action = [this, &next_message]() -> Action {
        absl::MutexLock status_lock(status_mutex_);
        // `kStart` is the only state that can initiate writes.
        if (status_ != ReactorStatus::kStarted) {
          return Action::kNone;
        }

        absl::MutexLock messages_lock(messages_mutex_);
        // Let's see if we have any messages to send.
        if (messages_.empty()) {
          // There are no more messages to send.
          // This can happen in two cases:
          //  1. gRPC is sending messages faster, than flow of `AddMessage`
          //  calls.
          //
          //     There is nothing to send, so we will exit early.
          //     Next `AddMessage` call will start the next write.
          //
          //  2. gRPC has sent all messages and `Finish` was called.
          //
          //     There will be no more messages at all and we shall finalize RPC
          //     call.
          //
          if (no_more_messages_) {
            // We are here because `Finish()` was called after the last chunk
            // was sent.
            //
            // All chunks of the message group were sent to the server, but
            // we don't know yet if it was delivered. There will be no more
            // writes from the client side, so we can finalize RPC call.
            status_ = ReactorStatus::kTerminating;
            return Action::kStartWritesDone;
          }

          // We still can send the data, but we don't have any.
          return Action::kNone;
        }
        status_ = ReactorStatus::kWriting;
        next_message = &messages_.front();
        return Action::kStartWrite;
      }();
    }

    switch (action) {
      case Action::kNone:
        return;
      case Action::kStartWrite:
        StartWrite(next_message);
        return;
      case Action::kStartWritesDone:
        on_message_group_sent_callback_(message_group_id_);
        // Then finalize the RPC call. This might trigger `OnDone` almost
        // immediately in different thread which then will trigger destruction
        // of `this`.
        StartWritesDone();
        return;
    }
  }

 private:
  SplitEngineMMDesktopBridge::Stub& stub_;
  const BridgeId bridge_id_;
  const MessageGroupId message_group_id_;
  const size_t message_group_max_size_bytes_;
  const MessageGroupSentCallback on_message_group_sent_callback_;

  absl::Notification done_;

  absl::Mutex messages_mutex_;
  std::queue<SendMessageGroupRequest> messages_
      ABSL_GUARDED_BY(messages_mutex_);
  bool no_more_messages_ ABSL_GUARDED_BY(messages_mutex_) = false;
  bool GetNoMoreMessages() {
    absl::MutexLock lock(messages_mutex_);
    return no_more_messages_;
  }
  void SetNoMoreMessages() {
    absl::MutexLock lock(messages_mutex_);
    no_more_messages_ = true;
  }

  absl::Mutex status_mutex_;
  ReactorStatus status_ ABSL_GUARDED_BY(status_mutex_) = ReactorStatus::kIdle;

  grpc::ClientContext context_;
  SendMessageGroupResponse response_;
};
}  // namespace

SplitEngineMMDesktopBridgeClientImpl::SplitEngineMMDesktopBridgeClientImpl(
    std::unique_ptr<SplitEngineMMDesktopBridge::Stub> stub)
    : client_id_(reinterpret_cast<ClientId>(this)), stub_(std::move(stub)) {
  

  struct {
    grpc::ClientContext context;
    ConnectRequest request;
    ConnectResponse response;
  } connect_args;

  absl::Notification notification;

  stub_->async()->Connect(
      &connect_args.context, &connect_args.request, &connect_args.response,
      [this, &notification, &connect_args](grpc::Status status) {
        
        bridge_id_ = connect_args.response.bridge_id();
        max_outstanding_message_groups_ =
            connect_args.response.max_outstanding_message_groups();
        notification.Notify();
      });
  notification.WaitForNotification();

  message_group_completion_reactor_ =
      std::make_unique<MessageGroupCompletionClientReactor>(
          bridge_id_,
          // Tell reactor how to call the RPC.
          [this](grpc::ClientContext* context,
                 MessageGroupCompletionRequest* request,
                 grpc::ClientReadReactor<MessageGroupCompletionResponse>*
                     reactor) {
            stub_->async()->ReadMessageGroupCompletions(context, request,
                                                        reactor);
          },
          // Tell reactor what to do when the server is done with a message
          // group.
          [this](MessageGroupId message_group_id) {
            IMP_LOG(imp::INFO) << "releasing message group " << message_group_id;
            

            SendMessageGroupReactor* reactor = nullptr;
            {
              absl::MutexLock lock(message_group_data_mutex_);
              auto it = message_group_data_.find(message_group_id);
              

              reactor = static_cast<SendMessageGroupReactor*>(
                  it->second.send_message_group_reactor.get());
            }

            if (reactor != nullptr) {
              // gRPC might not have called `OnDone` yet, so we need to wait for
              // it to make sure that we delete the reactor only after gRPC is
              // done with it.
              reactor->Await();
            }

            {
              absl::MutexLock lock(message_group_data_mutex_);
              message_group_data_.erase(message_group_id);
            }
          });

  heartbeat_thread_ = std::thread(
      &SplitEngineMMDesktopBridgeClientImpl::Heartbeat, this,
      absl::Milliseconds(connect_args.response.heartbeat_interval_ms()));
}

void SplitEngineMMDesktopBridgeClientImpl::Heartbeat(
    const absl::Duration heartbeat_interval) {
  struct HeartbeatRequestArgs {
    grpc::ClientContext context;
    HeartbeatRequest request;
    google::rpc::Status response;
  };

  absl::MutexLock lock(heartbeat_mutex_);
  while (!heartbeat_mutex_.AwaitWithTimeout(absl::Condition(
                                                +[](HeartbeatState* state) {
                                                  return *state ==
                                                         HeartbeatState::kStop;
                                                },
                                                &heartbeat_state_),
                                            heartbeat_interval)) {
    auto args = std::make_shared<HeartbeatRequestArgs>();
    args->request.set_bridge_id(bridge_id_);
    IMP_LOG(imp::INFO) << "Heartbeat " << bridge_id_;
    const absl::Time now = absl::Now();
    stub_->async()->Heartbeat(&args->context, &args->request, &args->response,
                              [args, now](grpc::Status status) {
                                
                                IMP_LOG(imp::INFO) << "Heartbeat roundtrip: "
                                           << absl::Now() - now;
                              });
  }
}

SplitEngineMMDesktopBridgeClientImpl::~SplitEngineMMDesktopBridgeClientImpl() {
  {
    absl::MutexLock lock(heartbeat_mutex_);
    heartbeat_state_ = HeartbeatState::kStop;
  }

  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }

  // Need to make sure that reactor is destructed before the client is removed
  // from the SplitEngineBridgeSender.
  message_group_completion_reactor_.reset();

  
}

absl::Status SplitEngineMMDesktopBridgeClientImpl::SendMessage(
    MessageGroupId message_group_id, size_t message_group_max_size_bytes,
    absl::Span<const uint8_t> data) {
  SendMessageGroupReactor* reactor = nullptr;
  bool start_call = false;
  {
    absl::MutexLock lock(message_group_data_mutex_);
    start_call = message_group_order_.empty();
    auto it = message_group_data_.find(message_group_id);
    if (it == message_group_data_.end()) {
      // New message group.
      std::unique_ptr<grpc::ClientWriteReactor<SendMessageGroupRequest>>
          send_message_group_reactor =
              std::make_unique<SendMessageGroupReactor>(
                  *stub_, bridge_id_, message_group_id,
                  message_group_max_size_bytes,
                  // Tell reactor what to do when it is done sending the message
                  // group.
                  [this](MessageGroupId message_group_id) {
                    SendMessageGroupReactor* reactor = nullptr;
                    {
                      absl::MutexLock lock(message_group_data_mutex_);
                      message_group_order_.pop_front();

                      if (!message_group_order_.empty()) {
                        // Need to find next message group ID and start the
                        // reactor.
                        MessageGroupId next_message_group_id =
                            message_group_order_.front();
                        auto it =
                            message_group_data_.find(next_message_group_id);
                        
                        reactor = static_cast<SendMessageGroupReactor*>(
                            it->second.send_message_group_reactor.get());
                      }
                    }
                    if (reactor != nullptr) {
                      reactor->Start();
                    }
                  });

      reactor = static_cast<SendMessageGroupReactor*>(
          send_message_group_reactor.get());

      message_group_order_.push_back(message_group_id);
      message_group_data_.emplace(
          message_group_id,
          MessageGroupData{.max_size_bytes = message_group_max_size_bytes,
                           .sent_bytes = data.size(),
                           .send_message_group_reactor =
                               std::move(send_message_group_reactor)});
    } else {
      // Check invariants.
      if (it->second.max_size_bytes != message_group_max_size_bytes) {
        return absl::InvalidArgumentError(
            "Message group max size bytes mismatch.");
      }
      it->second.sent_bytes += data.size();

      if (it->second.sent_bytes > it->second.max_size_bytes) {
        
        return absl::InvalidArgumentError(
            "Message group max size bytes exceeded.");
      }

      reactor = static_cast<SendMessageGroupReactor*>(
          it->second.send_message_group_reactor.get());
    }
  }

  if (start_call) {
    // Initiate the RPC
    reactor->Start();
  }

  reactor->AddMessage(data);

  return absl::OkStatus();
}

absl::Status SplitEngineMMDesktopBridgeClientImpl::EndMessageGroup(
    MessageGroupId message_group_id) {
  SendMessageGroupReactor* reactor = nullptr;

  {
    absl::MutexLock lock(message_group_data_mutex_);
    auto it = message_group_data_.find(message_group_id);
    if (it == message_group_data_.end()) {
      return absl::NotFoundError("Message group not found.");
    }
    reactor = static_cast<SendMessageGroupReactor*>(
        it->second.send_message_group_reactor.get());
  }

  // No more messages will be sent, so finalize the RPC call.
  reactor->Finish();
  return absl::OkStatus();
}

absl::Status SplitEngineMMDesktopBridgeClientImpl::SendRequest(
    absl::Span<const uint8_t> data,
    imp::Invocable<void(absl::Span<const uint8_t>)> callback) {
  struct SendRequestArgs {
    grpc::ClientContext context;
    SendRequestRequest request;
    SendRequestResponse response;
  };

  // Using shared_ptr to guarantee that the args will be destroyed even if the
  // gRPC callback is never called.
  auto args = std::make_shared<SendRequestArgs>();

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
      [callback_ptr = std::move(callback_ptr), args](grpc::Status status) {
        
        (*callback_ptr)(absl::MakeConstSpan(
            reinterpret_cast<const uint8_t*>(args->response.data().data()),
            args->response.data().size()));
      });

  return absl::OkStatus();
}

absl::Status
SplitEngineMMDesktopBridgeClientImpl::SetOnMessageGroupSentCallback(
    MessageGroupSentCallback&& callback) {
  on_message_group_sent_callback_ = std::move(callback);
  return absl::OkStatus();
}
void SplitEngineMMDesktopBridgeClientImpl::StopHeartbeat() {
  absl::MutexLock lock(heartbeat_mutex_);
  heartbeat_state_ = HeartbeatState::kStop;
}
}  // namespace imp::split_engine
