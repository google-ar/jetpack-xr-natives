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

#include "core/editor/widgets/performance/imgui_helper.h"

#include "dear_imgui/imgui.h"
#include "implot/implot.h"

namespace imp::editor {
void ImGuiHelper::DrawLegendItem(const char* label, bool& show_flag,
                                 int color_index) {
  ImVec2 swatch_size(10, 10);
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  ImVec2 p = ImGui::GetCursorScreenPos();
  ImVec4 color_vec = ImPlot::GetColormapColor(color_index);
  if (!show_flag) {
    color_vec.w *= 0.5f;  // Dim color if not shown
  }
  ImU32 col = ImGui::ColorConvertFloat4ToU32(color_vec);

  draw_list->AddRectFilled(p, ImVec2(p.x + swatch_size.x, p.y + swatch_size.y),
                           col);
  ImGui::Dummy(swatch_size);
  ImGui::SameLine();

  bool faded = false;
  if (!show_flag) {
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    faded = true;
  }

  // Vertically center the text next to the swatch
  float text_height = ImGui::GetTextLineHeight();
  float offset_y = (swatch_size.y - text_height) * 0.5f;
  ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset_y);

  ImGui::Text(label);
  if (ImGui::IsItemClicked()) {
    show_flag = !show_flag;
  }
  if (faded) {
    ImGui::PopStyleVar();
  }
}

}  // namespace imp::editor
