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

#include "core/split_engine/desktop/utils/message_group_completion_client_reactor.h"

#include <utility>

#include "absl/log/check.h"
#include "third_party/grpc/include/grpcpp/support/status.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

MessageGroupCompletionClientReactor::MessageGroupCompletionClientReactor(
    BridgeId bridge_id,
    ReadMessageGroupCompletionsFunc&& read_message_group_completions_func,
    OnMessageGroupProcessedCallback&& on_message_group_processed_callback)
    : on_message_group_processed_callback_(
          std::move(on_message_group_processed_callback)) {
  request_.set_bridge_id(bridge_id);

  read_message_group_completions_func(&context_, &request_, this);

  StartCall();

  StartRead(&response_);
}

void MessageGroupCompletionClientReactor::OnReadDone(bool ok) {
  if (!ok) {
    return;
  }

  on_message_group_processed_callback_(response_.message_group_id());

  response_.Clear();
  StartRead(&response_);
}

void MessageGroupCompletionClientReactor::OnDone(const grpc::Status& status) {
  
}
}  // namespace imp::split_engine
