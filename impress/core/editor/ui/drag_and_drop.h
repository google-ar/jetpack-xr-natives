/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DRAG_AND_DROP_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DRAG_AND_DROP_H_

#include <optional>
#include <string>

#include "absl/strings/ascii.h"
#include "absl/strings/string_view.h"
#include "core/common/type_traits.h"

namespace imp::editor {

// The set of all standard drag-and-drop payload types for the editor.
enum class DragAndDropType {
  kNode,
  kNodeAsset,
  kMaterial,
  kTexture,
};

// Combo ImGui BeginDragAndDropSource + SetDragAndDropPayload + DragAndDropType.
bool BeginDragAndDropSource(DragAndDropType drag_and_drop_type,
                            absl::string_view label, absl::string_view payload);

// Mirror of ImGui::SetDragAndDropPayload w/ DragAndDropType support.
void SetDragAndDropPayload(DragAndDropType drag_and_drop_type,
                           absl::string_view label, absl::string_view payload);

// Mirror of ImGui::AcceptDragAndDropPayload w/ DragAndDropType support.
std::optional<std::string> AcceptDragAndDropPayload(
    DragAndDropType drag_and_drop_type);
std::optional<std::string> AcceptDragAndDropPayload(
    absl::string_view drag_and_drop_type);

// Gets the drag-and-drop ID string for the given type (for accepting payloads).
absl::string_view GetDragAndDropTypeId(DragAndDropType drag_and_drop_type);

template <typename T>
std::string GetPayloadTypeForMessage() {
  std::string payload_type = absl::AsciiStrToLower(type_traits::kTypeName<T>);
  const std::string s = "::";
  const std::string t = ".";
  std::string::size_type n = 0;
  while ((n = payload_type.find(s, n)) != std::string::npos) {
    payload_type.replace(n, s.size(), t);
    n += t.size();
  }
  return payload_type;
}

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DRAG_AND_DROP_H_
