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

#include "core/editor/ui/drag_and_drop_node.h"

#include <string>

#include "dear_imgui/imgui.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

bool BeginDragAndDropSource(NodeHandle node) {
  if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
    SetDragAndDropPayload(node);
    ImGui::EndDragDropSource();
    return true;
  }
  return false;
}

void SetDragAndDropPayload(NodeHandle node) {
  int entity_id = node->GetEntity().getId();
  ImGui::SetDragDropPayload(
      std::string(GetDragAndDropTypeId(DragAndDropType::kNode)).c_str(),
      &entity_id, sizeof(int));
}

NodeHandle GetDragAndDropPayloadNode() {
  const ImGuiPayload* payload = ImGui::GetDragDropPayload();
  if (!payload) {
    return {};
  }

  if (!payload->IsDataType(
          std::string(GetDragAndDropTypeId(DragAndDropType::kNode)).c_str())) {
    return {};
  }

  int entityId = *static_cast<int*>(payload->Data);
  return NodeHandle(utils::Entity::import(entityId));
}

NodeHandle AcceptDragAndDropPayloadNode() {
  if (ImGui::AcceptDragDropPayload(
          std::string(GetDragAndDropTypeId(DragAndDropType::kNode)).c_str())) {
    int entityId = *static_cast<int*>(ImGui::GetDragDropPayload()->Data);
    return NodeHandle(utils::Entity::import(entityId));
  }
  return NodeHandle();
}

}  // namespace imp::editor
