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

ImVec4 WithAlpha(const ImVec4& color, float alpha) {
  return ImVec4(color.x, color.y, color.z, alpha);
}

void SetupEditorImGuiStyle() {
  ImGuiStyle& style = ImGui::GetStyle();
  ImVec4* colors = style.Colors;

  style.FrameRounding = 3.0f;
  style.WindowRounding = 3.0f;

  colors[ImGuiCol_Text] = kDarkTextGrey;
  colors[ImGuiCol_TextDisabled] = kDarkTextGreyAlt;
  colors[ImGuiCol_WindowBg] = kDarkSurface;

  colors[ImGuiCol_ChildBg] = kCommonTransparent;
  colors[ImGuiCol_Border] = kDarkOutline;
  colors[ImGuiCol_BorderShadow] = kCommonTransparent;

  colors[ImGuiCol_FrameBg] = kDarkField;
  colors[ImGuiCol_FrameBgHovered] = kDarkField;
  colors[ImGuiCol_FrameBgActive] = kDarkField;

  colors[ImGuiCol_TitleBg] = kDarkSurface;
  colors[ImGuiCol_TitleBgActive] = kDarkSurface;
  colors[ImGuiCol_TitleBgCollapsed] = kDarkSurface;

  colors[ImGuiCol_TabHovered] = kDarkLowlight;

  colors[ImGuiCol_Tab] = kDarkTextGreyAlt;
  colors[ImGuiCol_TabSelected] = kDarkSurface;
  colors[ImGuiCol_TabSelectedOverline] = kDarkOutline;

  colors[ImGuiCol_TabDimmed] = kDarkTextGreyAlt;
  colors[ImGuiCol_TabDimmedSelected] = kDarkSurface;
  colors[ImGuiCol_TabDimmedSelectedOverline] = kDarkTextGreyAlt;

  colors[ImGuiCol_Header] = kDarkSurface;
  colors[ImGuiCol_HeaderHovered] = kDarkSurface;
  colors[ImGuiCol_HeaderActive] = kDarkSurface;

  colors[ImGuiCol_CheckMark] = kDarkTextGrey;

  colors[ImGuiCol_Button] = kDarkPrimary;
  colors[ImGuiCol_ButtonHovered] = kDarkLowlight;
  colors[ImGuiCol_ButtonActive] = kDarkSelectHighlight;

  colors[ImGuiCol_SliderGrab] = kDarkPrimary;
  colors[ImGuiCol_SliderGrabActive] = kDarkSelectHighlight;
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
