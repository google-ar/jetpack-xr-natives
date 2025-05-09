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

#include "core/scene_handles/scene_handle_helper.h"

#include <cstdint>
#include <string>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/variant.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/scene_handles/scene_handle_interface.h"
#include "core/view/framework/scene/scene_identifier.h"

namespace imp {

SceneHandleHelper::SceneHandleHelper() {}

SceneHandleHelper::SceneHandleHelper(Identifier identifier)
    : identifier_(identifier) {}

SceneHandleInterface::Identifier& SceneHandleHelper::GetIdentifier() {
  return identifier_;
}

std::string SceneHandleHelper::GetIdentifierString() const {
  if (absl::holds_alternative<absl::monostate>(identifier_)) {
    return "Empty";
  } else if (absl::holds_alternative<std::string>(identifier_)) {
    return absl::StrFormat("name=%s", absl::get<std::string>(identifier_));
  } else {
    return absl::StrFormat("unique_id=%i", absl::get<int32_t>(identifier_));
  }
}

absl::Status SceneHandleHelper::RequireIdentifiedNode(
    NodeHandle identified_node) const {
  if (!identified_node) {
    return absl::UnavailableError(
        absl::StrCat("SceneHandle cannot find node with identifier ",
                     GetIdentifierString()));
  }

  return absl::OkStatus();
}

void SceneHandleHelper::UpdateIdentifier(NodeHandle scene_node) {
  if (!scene_node) {
    return;
  }

  if (auto scene_identifier = scene_node->GetComponent<SceneIdentifier>()) {
    identifier_ = scene_identifier->GetId();
  } else if (!scene_node->GetName().empty()) {
    identifier_ = std::string(scene_node->GetName());
  } else {
    IMP_LOG(imp::WARNING)
        << "Unable to update identifier for node, it must have either a "
           "name or a SceneIdentifier.";
  }
}

}  // namespace imp
