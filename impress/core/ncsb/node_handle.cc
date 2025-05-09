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

#include "core/ncsb/node_handle.h"

#include <string>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/strings/str_format.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/EntityManager.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_controller.h"
#include "core/view/base_view.h"

namespace imp {

NodeHandle::NodeHandle() : node_(utils::Entity()) {}

NodeHandle::NodeHandle(utils::Entity entity) : node_(entity) {}

NodeHandle::NodeHandle(const Node& node) : node_(node) {}

NodeHandle::NodeHandle(utils::Entity entity,
                       imp_internal::NodeController* node_controller)
    : node_(entity, node_controller) {}

Node& NodeHandle::operator*() const noexcept { return *operator->(); }

Node* NodeHandle::operator->() const noexcept {
  AssertIsValid();
  return static_cast<Node*>(&node_);
}

bool NodeHandle::operator==(const NodeHandle& other) const {
  return node_ == other.node_;
}

bool NodeHandle::operator!=(const NodeHandle& other) const {
  return node_ != other.node_;
}

NodeHandle::operator bool() const noexcept { return IsValid(); }

bool NodeHandle::IsValid() const {
  utils::Entity entity = node_.GetEntity();
  if (!utils::EntityManager::get().isAlive(entity)) {
    // The contained value is currently not a valid filament entity.
    return false;
  }

  // If the entity is valid & alive, but the node controller is null, then the
  // filament entity isn't attached to the view and therefore isn't a node.
  //
  // If the node is destroyed, then the node_controller_ will be a dangling
  // pointer, but we will have returned already because the entity won't be
  // alive.
  return node_.node_controller_ != nullptr;
}

bool NodeHandle::IsDefaultValue() const { return node_.GetEntity().isNull(); }

void NodeHandle::AssertIsValid() const {
  

  // This has a mild cost to it, which accumulates over large numbers of calls
  // to NodeHandle. Disable in opt builds.
  
}

utils::Entity NodeHandle::GetEntity() const { return node_.GetEntity(); }

std::string ToString(const NodeHandle& handle) {
  if (handle.IsValid()) {
    if (auto name = handle->GetName(); !name.empty()) {
      return absl::StrFormat("Node<%s>#%d", name, handle->GetEntity().getId());
    }
    return absl::StrFormat("Node#%d", handle->GetEntity().getId());
  }
  if (handle.IsDefaultValue()) {
    return absl::StrFormat("Node(Null)");
  }
  return absl::StrFormat("Node(Destroyed)#%d", handle.GetEntity().getId());
}

}  // namespace imp
