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

#include "core/ncsb/dispatcher/event.h"

#include "core/common/platform_helpers.h"

namespace imp {

Event::Event()
    : mode_(kBubbleUp),
      connection_holder_(nullptr),
      connection_id_(kNullConnectionId) {}
Event::PropagationMode Event::GetPropagationMode() const { return mode_; }
NodeHandle Event::GetOriginatingNode() const { return originating_node_; }
NodeHandle Event::GetTargetNode() const { return target_node_; }
void Event::SetPropagationMode(PropagationMode mode) { mode_ = mode; }
void Event::SetOriginatingNode(NodeHandle node) { originating_node_ = node; }
void Event::SetTargetNode(NodeHandle node) { target_node_ = node; }
void Event::SetConnectionInfo(ConnectionHolder* connection_holder,
                              ConnectionId connection_id) {
  connection_holder_ = connection_holder;
  connection_id_ = connection_id;
}

void Event::Disconnect() const {
  if (connection_holder_) {
    connection_holder_->Disconnect(connection_id_);
  }
}

}  // namespace imp
