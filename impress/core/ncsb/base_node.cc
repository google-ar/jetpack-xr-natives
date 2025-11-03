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

#include "core/ncsb/base_node.h"

#include "absl/log/check.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/TransformManager.h"
#include "core/ncsb/node_attachment_manager.h"

namespace imp {

BaseNode::BaseNode(utils::Entity entity) : entity_(entity) {
  // If this entity is valid, then try to get the NodeController and cache it.
  if (!entity_.isNull()) {
    node_controller_ = imp_internal::NodeAttachmentManager::Get(entity_);
  }
}

BaseNode::BaseNode(utils::Entity entity,
                   imp_internal::NodeController* node_controller)
    : entity_(entity), node_controller_(node_controller) {
  
}

bool BaseNode::operator==(const BaseNode& other) const {
  return entity_ == other.entity_;
}

bool BaseNode::operator!=(const BaseNode& other) const {
  return entity_ != other.entity_;
}

}  // namespace imp
