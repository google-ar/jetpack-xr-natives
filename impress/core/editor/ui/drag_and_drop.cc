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

#include "core/editor/ui/drag_and_drop.h"

#include <optional>
#include <string>

#include "dear_imgui/imgui_internal.h"

namespace imp::editor {

absl::string_view GetDragAndDropTypeId(DragAndDropType drag_and_drop_type) {
  switch (drag_and_drop_type) {
    case DragAndDropType::kNode:
      return "kDragAndDropNode";
    case DragAndDropType::kNodeAsset:
      return "kDragAndDropNodeAsset";
    case DragAndDropType::kMaterial:
      return "kDragAndDropMaterialAsset";
    case DragAndDropType::kTexture:
      return "kDragAndDropTextureAsset";
  }
}

bool BeginDragAndDropSource(DragAndDropType drag_and_drop_type,
                            absl::string_view label,
                            absl::string_view payload) {
  if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceAllowNullID)) {
    SetDragAndDropPayload(drag_and_drop_type, label, payload);
    ImGui::EndDragDropSource();
    return true;
  }
  return false;
}

void SetDragAndDropPayload(DragAndDropType drag_and_drop_type,
                           absl::string_view label, absl::string_view payload) {
  ImGui::SetDragDropPayload(
      std::string(GetDragAndDropTypeId(drag_and_drop_type)).c_str(),
      payload.data(), payload.size());
  if (!label.empty()) {
    ImGui::Text(std::string(label).c_str());
  }
}

std::optional<std::string> AcceptDragAndDropPayload(
    DragAndDropType drag_and_drop_type) {
  if (ImGui::AcceptDragDropPayload(
          std::string(GetDragAndDropTypeId(drag_and_drop_type)).c_str())) {
    return std::string(static_cast<char*>(ImGui::GetDragDropPayload()->Data),
                       ImGui::GetDragDropPayload()->DataSize);
  } else {
    return std::nullopt;
  }
}

std::optional<std::string> AcceptDragAndDropPayload(
    absl::string_view drag_and_drop_type) {
  if (ImGui::AcceptDragDropPayload(std::string(drag_and_drop_type).c_str())) {
    return std::string(static_cast<char*>(ImGui::GetDragDropPayload()->Data),
                       ImGui::GetDragDropPayload()->DataSize);
  } else {
    return std::nullopt;
  }
}

}  // namespace imp::editor
