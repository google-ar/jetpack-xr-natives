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

#include "core/split_engine/desktop/utils/message_group_completion_server_reactor.h"

#include <utility>

#include "core/common/log.h"
#include "absl/synchronization/mutex.h"
#include "third_party/grpc/include/grpcpp/support/status.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

void MessageGroupCompletionServerReactor::ReportCompletion(
    MessageGroupId message_group_id) {
  {
    MessageGroupCompletionResponse response;
    response.set_message_group_id(message_group_id);

    IMP_LOG(imp::INFO) << "Reporting completion: " << response.DebugString();

    absl::MutexLock lock(responses_mutex_);
    responses_.push(std::move(response));
  }

  Write();
}

void MessageGroupCompletionServerReactor::Shutdown(const grpc::Status& status) {
  absl::MutexLock status_lock(status_mutex_);
  if (status_ == ReactorStatus::kDead ||
      status_ == ReactorStatus::kTerminating) {
    return;
  }

  status_ = ReactorStatus::kTerminating;
  Finish(status);
}

void MessageGroupCompletionServerReactor::OnDone() {
  absl::MutexLock status_lock(status_mutex_);
  status_ = ReactorStatus::kDead;
}

void MessageGroupCompletionServerReactor::OnCancel() {
  absl::MutexLock status_lock(status_mutex_);
  if (status_ == ReactorStatus::kDead ||
      status_ == ReactorStatus::kTerminating) {
    return;
  }
  status_ = ReactorStatus::kTerminating;
  Finish(
      grpc::Status(grpc::StatusCode::CANCELLED, "Client cancelled requests."));
}

void MessageGroupCompletionServerReactor::OnWriteDone(bool ok) {
  {
    absl::MutexLock status_lock(status_mutex_);
    if (!ok) {
      if (status_ != ReactorStatus::kTerminating &&
          status_ != ReactorStatus::kDead) {
        status_ = ReactorStatus::kTerminating;
        Finish(grpc::Status(grpc::StatusCode::INTERNAL,
                            "Failed to write response"));
      }
      return;
    }
    if (status_ != ReactorStatus::kWriting) {
      IMP_LOG(imp::ERROR) << "Successful OnWriteDone called when not writing.";
      return;
    }
    {
      absl::MutexLock events_lock(responses_mutex_);
      responses_.pop();
    }

    status_ = ReactorStatus::kIdle;
  }

  Write();
}

void MessageGroupCompletionServerReactor::Write() {
  absl::MutexLock status_lock(status_mutex_);
  if (status_ != ReactorStatus::kIdle) {
    return;
  }

  absl::MutexLock responses_lock(responses_mutex_);
  if (responses_.empty()) {
    return;
  }

  status_ = ReactorStatus::kWriting;

  StartWrite(&responses_.front());
}

}  // namespace imp::split_engine
