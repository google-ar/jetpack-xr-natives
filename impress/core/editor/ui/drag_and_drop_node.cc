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

#include <cstdint>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "dear_imgui/imgui.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

bool BeginDragAndDropSource(NodeHandle node) {
  return BeginDragAndDropSource(absl::MakeSpan(&node, 1));
}

bool BeginDragAndDropSource(absl::Span<const NodeHandle> nodes) {
  if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
    SetDragAndDropPayload(nodes);
    ImGui::EndDragDropSource();
    return true;
  }
  return false;
}

void SetDragAndDropPayload(NodeHandle node) {
  SetDragAndDropPayload(std::vector<NodeHandle>{node});
}

void SetDragAndDropPayload(absl::Span<const NodeHandle> nodes) {
  std::vector<uint32_t> entity_ids;
  entity_ids.reserve(nodes.size());
  for (const auto& node : nodes) {
    entity_ids.push_back(node->GetEntity().getId());
  }

  // ImGui::SetDragDropPayload copies the data, so it's safe to pass a pointer
  // to the local `entity_ids` vector. That also means we can only pass in
  // trivially copyable types.
  ImGui::SetDragDropPayload(
      std::string(GetDragAndDropTypeId(DragAndDropType::kNode)).c_str(),
      entity_ids.data(), entity_ids.size() * sizeof(uint32_t));
}

NodeHandle GetDragAndDropPayloadNode() {
  std::vector<NodeHandle> nodes = GetDragAndDropPayloadNodes();
  if (nodes.empty()) {
    return {};
  }

  return nodes[0];
}

std::vector<NodeHandle> GetDragAndDropPayloadNodes() {
  const ImGuiPayload* payload = ImGui::GetDragDropPayload();
  if (!payload) {
    return {};
  }

  if (!payload->IsDataType(
          std::string(GetDragAndDropTypeId(DragAndDropType::kNode)).c_str())) {
    return {};
  }

  absl::Span<uint32_t> entity_ids(static_cast<uint32_t*>(payload->Data),
                                  payload->DataSize / sizeof(uint32_t));
  std::vector<NodeHandle> nodes;
  nodes.reserve(entity_ids.size());
  for (uint32_t entity_id : entity_ids) {
    nodes.push_back(NodeHandle(utils::Entity::import(entity_id)));
  }
  return nodes;
}

NodeHandle AcceptDragAndDropPayloadNode() {
  std::vector<NodeHandle> nodes = AcceptDragAndDropPayloadNodes();
  if (nodes.empty()) {
    return {};
  }

  return nodes[0];
}

std::vector<NodeHandle> AcceptDragAndDropPayloadNodes() {
  std::vector<NodeHandle> nodes;
  if (const ImGuiPayload* payload = ImGui::AcceptDragDropPayload(
          std::string(GetDragAndDropTypeId(DragAndDropType::kNode)).c_str())) {
    absl::Span<uint32_t> entity_ids(static_cast<uint32_t*>(payload->Data),
                                    payload->DataSize / sizeof(uint32_t));
    nodes.reserve(entity_ids.size());
    for (uint32_t entity_id : entity_ids) {
      nodes.push_back(NodeHandle(utils::Entity::import(entity_id)));
    }
  }

  return nodes;
}

}  // namespace imp::editor
