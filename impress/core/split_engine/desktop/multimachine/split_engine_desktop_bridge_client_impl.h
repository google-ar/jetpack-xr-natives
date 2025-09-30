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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_CLIENT_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_CLIENT_IMPL_H_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <thread>  // NOLINT: Need to use threads available in bazel.
#include <utility>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "third_party/grpc/include/grpcpp/support/client_callback.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge.grpc.pb.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_client.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

class SplitEngineMMDesktopBridgeClientImpl
    : public SplitEngineMMDesktopBridgeClient {
 public:
  explicit SplitEngineMMDesktopBridgeClientImpl(
      std::unique_ptr<SplitEngineMMDesktopBridge::Stub> stub);
  ~SplitEngineMMDesktopBridgeClientImpl() override;

  absl::Status SetOnMessageGroupSentCallback(
      MessageGroupSentCallback&& callback) override;

  absl::Status SendRequest(
      absl::Span<const uint8_t> data,
      std::function<void(absl::Span<const uint8_t>)> callback) override;

  absl::Status SendMessage(MessageGroupId message_group_id,
                           size_t message_group_max_size_bytes,
                           absl::Span<const uint8_t> data) override;

  absl::Status EndMessageGroup(MessageGroupId message_group_id) override;

  MessageGroupId GenerateMessageGroupId() override {
    static MessageGroupId message_group_id = 0;
    return ++message_group_id;
  }

  ClientId GetClientId() const override { return client_id_; }

 private:
  static constexpr size_t kRpcStreamChunkSizeBytes = 64 * 1024;
  const ClientId client_id_;
  BridgeId bridge_id_;
  size_t max_outstanding_message_groups_;
  std::unique_ptr<SplitEngineMMDesktopBridge::Stub> stub_;
  std::unique_ptr<grpc::ClientReadReactor<MessageGroupCompletionResponse>>
      message_group_completion_reactor_;

  size_t active_message_group_size_bytes_ = 0;
  MessageGroupSentCallback on_message_group_sent_callback_;

  struct MessageGroupData {
    size_t max_size_bytes;
    size_t sent_bytes;
    std::unique_ptr<grpc::ClientWriteReactor<SendMessageGroupRequest>>
        send_message_group_reactor;
  };

  absl::Mutex message_group_data_mutex_;
  std::deque<MessageGroupId> message_group_order_
      ABSL_GUARDED_BY(message_group_data_mutex_);
  absl::flat_hash_map<MessageGroupId, MessageGroupData> message_group_data_
      ABSL_GUARDED_BY(message_group_data_mutex_);

  std::thread heartbeat_thread_;

  bool stop_heartbeat_ = false;
  void Heartbeat(absl::Duration heartbeat_interval);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_MULTIMACHINE_SPLIT_ENGINE_DESKTOP_BRIDGE_CLIENT_IMPL_H_
