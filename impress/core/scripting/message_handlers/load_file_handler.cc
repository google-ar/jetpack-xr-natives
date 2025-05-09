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

#include "core/scripting/message_handlers/load_file_handler.h"

#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/editor/file_loader_helper.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/scripting/message_helpers.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/view_events.h"

namespace imp::scripting {

LoadFileHandler::LoadFileHandler(BaseView& base_view) : view_(base_view) {}

Future<absl::Status> LoadFileHandler::HandleMessage(
    const LoadFileRequest& message) {
  std::string deserialized_result;
  if (!DeserializeBase64(message.file_contents, &deserialized_result)) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        absl::StrFormat("Failed to decode base64 string received from JS: %s",
                        message.file_contents)));
  }

  view_.GetDispatcher().Send(imp::DropFileEvent(
      message.file_id, absl::Cord(std::move(deserialized_result))));

  return Future<absl::Status>(absl::OkStatus());
}

}  // namespace imp::scripting
