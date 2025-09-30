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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_MESSAGE_GROUP_COMPLETION_SERVER_REACTOR_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_MESSAGE_GROUP_COMPLETION_SERVER_REACTOR_H_

#include <cstdint>
#include <queue>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "third_party/grpc/include/grpcpp/support/server_callback.h"
#include "third_party/grpc/include/grpcpp/support/status.h"
#include "core/split_engine/desktop/split_engine_desktop_bridge.proto.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// Supports sending message group completions via gRPC stream on a server.
class MessageGroupCompletionServerReactor final
    : public grpc::ServerWriteReactor<MessageGroupCompletionResponse> {
  enum class ReactorStatus : uint8_t {
    kIdle,         // No write is in progress.
    kWriting,      // Write is in progress.
    kTerminating,  // Reactor is being terminated.
    kDead,         // Reactor is no longer active.
  };

 public:
  MessageGroupCompletionServerReactor() = default;
  ~MessageGroupCompletionServerReactor() override = default;

  // Adds a message group completion to the queue of responses to be sent.
  void ReportCompletion(MessageGroupId message_group_id);

  // Terminates the stream with the given status.
  void Shutdown(const grpc::Status& status);

  void OnDone() override;

  void OnCancel() override;

  void OnWriteDone(bool ok) override;

 private:
  void Write();

  absl::Mutex responses_mutex_;
  std::queue<MessageGroupCompletionResponse> responses_
      ABSL_GUARDED_BY(responses_mutex_);

  absl::Mutex status_mutex_;
  ReactorStatus status_ ABSL_GUARDED_BY(status_mutex_) = ReactorStatus::kIdle;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_DESKTOP_UTILS_MESSAGE_GROUP_COMPLETION_SERVER_REACTOR_H_
