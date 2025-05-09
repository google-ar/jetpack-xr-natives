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

#include "core/editor/widgets/recipes/editor_helpers.h"

#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "core/editor/widgets/recipes/icons.h"

namespace imp::editor {

void DrawRecipeIcon(const ImVec2& size, RecipeIconType type, bool filled,
                    const ImVec4& color, const ImVec4& innerColor) {
  if (ImGui::IsRectVisible(size)) {
    auto cursorPos = ImGui::GetCursorScreenPos();
    auto drawList = ImGui::GetWindowDrawList();
    recipe_internal::DrawIcon(drawList, cursorPos, cursorPos + size, type,
                              filled, ImColor(color), ImColor(innerColor));
  }

  ImGui::Dummy(size);
}

}  // namespace imp::editor
