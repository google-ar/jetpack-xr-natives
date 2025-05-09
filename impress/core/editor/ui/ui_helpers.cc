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

#include "core/editor/ui/ui_helpers.h"

#include "dear_imgui/imgui.h"

namespace imp::editor {

void ImGuiCenterNextHorizontally(float next_width,
                                 IncludePadding include_padding) {
  float size = next_width;
  if (include_padding == IncludePadding::kCell) {
    size += ImGui::GetStyle().CellPadding.x * 2.0f;
  }

  float avail = ImGui::GetContentRegionAvail().x;
  float offset = (avail - size) * 0.5f;

  if (offset > 0.0f) {
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
  }
}

}  // namespace imp::editor
