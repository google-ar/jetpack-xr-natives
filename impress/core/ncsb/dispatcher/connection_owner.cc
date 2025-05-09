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

#include "core/ncsb/dispatcher/connection_owner.h"

#include <cstddef>

#include "core/ncsb/component.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/node_handle.h"

namespace imp {

ConnectionOwner::ConnectionOwner(Component* component)
    : pointer_owner_(nullptr),
      node_(component->GetNode()),
      component_id_(component->GetComponentId()) {}

ConnectionOwner::ConnectionOwner(NodeHandle node)
    : pointer_owner_(nullptr), node_(node), component_id_() {}

ConnectionOwner::ConnectionOwner(void* pointer_owner)
    : pointer_owner_(pointer_owner), node_(), component_id_() {}

ConnectionOwner::ConnectionOwner(std::nullptr_t) : ConnectionOwner() {}

ConnectionOwner::ConnectionOwner()
    : pointer_owner_(nullptr), node_(), component_id_() {}

void* ConnectionOwner::GetPointerOwner() const { return pointer_owner_; }

NodeHandle ConnectionOwner::GetNode() const { return node_; }

ComponentId ConnectionOwner::GetComponentId() const { return component_id_; }

bool ConnectionOwner::IsValid() const {
  return !node_.GetEntity().isNull() || (GetPointerOwner() != nullptr);
}

bool ConnectionOwner::operator==(const ConnectionOwner& other) const {
  return pointer_owner_ == other.pointer_owner_ && node_ == other.node_ &&
         component_id_ == other.component_id_;
}

bool ConnectionOwner::operator!=(const ConnectionOwner& other) const {
  return !(*this == other);
}

}  // namespace imp
