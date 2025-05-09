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

#include "core/scripting/message_handlers/set_node_name_handler.h"

#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/scripting/proto/api.proto.imp.h"

namespace imp::scripting {

Future<absl::Status> SetNodeNameHandler::HandleMessage(
    const SetNodeNameRequest& message) {
  if (!message.target) {
    return Future<absl::Status>(absl::InvalidArgumentError("Invalid Node ID"));
  }
  message.target->SetName(message.name);
  return Future<absl::Status>(absl::OkStatus());
}

}  // namespace imp::scripting
