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

#include "core/scripting/message_handlers/get_children_handler.h"

#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/base_view.h"

namespace imp::scripting {

Future<NodeListResponse> GetChildrenHandler::HandleMessage(
    const GetChildrenRequest& message) {
  NodeListResponse response;
  if (!message.target) {
    return Future<NodeListResponse>(absl::InvalidArgumentError("Invalid Node"));
  }
  response.nodes = message.target->GetChildren();
  return Future<NodeListResponse>(std::move(response));
}

}  // namespace imp::scripting
