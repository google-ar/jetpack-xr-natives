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

#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/ncsb/node.h"
#include "core/view/base_view.h"

namespace imp {

NodeHandle::NodeHandle(utils::Entity entity) : node_(entity) {}

bool NodeHandle::IsValid() const {
  utils::Entity entity = node_.GetEntity();

  imp_internal::NodeController* lookup =
      imp_internal::NodeAttachmentManager::Get(entity);

  return lookup != nullptr && lookup == node_.node_controller_ &&
         lookup->GetEntity() == entity;
}

std::string ToString(const NodeHandle& handle) {
  if (handle.IsValid()) {
    if (absl::string_view name = handle->GetName(); !name.empty()) {
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
