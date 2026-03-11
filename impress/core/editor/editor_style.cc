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

#include "core/editor/editor_style.h"

#include "dear_imgui/imgui.h"

namespace imp::editor {

namespace {

ImVec4 ImLerp(const ImVec4& a, const ImVec4& b, float t) {
  return ImVec4(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t);
}

}  // namespace

ImVec4 WithAlpha(const ImVec4& color, float alpha) {
  return ImVec4(color.x, color.y, color.z, alpha);
}

void SetupEditorImGuiStyle() {
  ImGuiStyle& style = ImGui::GetStyle();
  ImVec4* colors = style.Colors;

  style.FrameRounding = 3.0f;
  style.WindowRounding = 3.0f;

  colors[ImGuiCol_Text] = kWhite;
  colors[ImGuiCol_TextDisabled] = kGrey600;
  colors[ImGuiCol_WindowBg] = WithAlpha(kGrey900, 0.94f);
  colors[ImGuiCol_ChildBg] = kTransparent;
  colors[ImGuiCol_Border] = kGrey700;
  colors[ImGuiCol_BorderShadow] = kTransparent;
  colors[ImGuiCol_FrameBg] = WithAlpha(kBlue900, 0.8f);
  colors[ImGuiCol_FrameBgHovered] = WithAlpha(kBlue900, 0.9f);
  colors[ImGuiCol_FrameBgActive] = kBlue900;
  colors[ImGuiCol_TitleBg] = kGrey800;
  colors[ImGuiCol_TitleBgActive] = kGrey900;
  colors[ImGuiCol_TitleBgCollapsed] = WithAlpha(kGrey900, 0.5f);
  colors[ImGuiCol_CheckMark] = kWhite;
  colors[ImGuiCol_Button] = WithAlpha(kBlue700, 0.8f);
  colors[ImGuiCol_ButtonHovered] = WithAlpha(kBlue700, 0.9f);
  colors[ImGuiCol_ButtonActive] = kBlue700;
  colors[ImGuiCol_Header] = WithAlpha(kBlue600, 0.8f);
  colors[ImGuiCol_HeaderHovered] = WithAlpha(kBlue600, 0.9f);
  colors[ImGuiCol_HeaderActive] = kBlue600;
  colors[ImGuiCol_Tab] = WithAlpha(kBlue600, 0.8f);
  colors[ImGuiCol_TabHovered] = WithAlpha(kBlue600, 0.9f);
  colors[ImGuiCol_TabSelected] =
      ImLerp(colors[ImGuiCol_Tab], colors[ImGuiCol_TitleBg], 0.5f);
}

void PushBaseIsfElementStyle() {
  ImGui::PushStyleColor(ImGuiCol_FrameBg, WithAlpha(kGrey500, 0.5f));
  ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, WithAlpha(kGrey500, 0.7f));
  ImGui::PushStyleColor(ImGuiCol_FrameBgActive, kGrey500);
  ImGui::PushStyleColor(ImGuiCol_Button, WithAlpha(kGrey700, 0.8f));
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, WithAlpha(kGrey700, 0.9f));
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, kGrey700);
  ImGui::PushStyleColor(ImGuiCol_Text, kGrey400);
  ImGui::PushStyleColor(ImGuiCol_CheckMark, kGrey400);
}

void PopBaseIsfElementStyle() { ImGui::PopStyleColor(8); }

}  // namespace imp::editor
