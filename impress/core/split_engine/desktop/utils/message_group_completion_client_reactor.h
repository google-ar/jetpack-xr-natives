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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_MESSAGE_GROUP_COMPLETION_REACTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_MESSAGE_GROUP_COMPLETION_REACTOR_H_

#include <functional>
#include <utility>

#include "absl/log/check.h"
#include "third_party/grpc/include/grpcpp/client_context.h"
#include "third_party/grpc/include/grpcpp/support/client_callback.h"
#include "third_party/grpc/include/grpcpp/support/status.h"
#include "core/split_engine/desktop/split_engine_desktop_bridge.proto.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// Supports receiving message group completions via gRPC stream on a client.
class MessageGroupCompletionClientReactor final
    : public grpc::ClientReadReactor<MessageGroupCompletionResponse> {
 public:
  using ReadMessageGroupCompletionsFunc = std::function<void(
      grpc::ClientContext*, MessageGroupCompletionRequest*,
      grpc::ClientReadReactor<MessageGroupCompletionResponse>*)>;

  using OnMessageGroupProcessedCallback = std::function<void(MessageGroupId)>;

  MessageGroupCompletionClientReactor(
      BridgeId bridge_id,
      ReadMessageGroupCompletionsFunc&& read_message_group_completions_func,
      OnMessageGroupProcessedCallback&& on_message_group_processed_callback);

  void OnReadDone(bool ok) override;

  void OnDone(const grpc::Status& status) override;

 private:
  const OnMessageGroupProcessedCallback on_message_group_processed_callback_;
  grpc::ClientContext context_;
  MessageGroupCompletionRequest request_;
  MessageGroupCompletionResponse response_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_MESSAGE_GROUP_COMPLETION_REACTOR_H_
