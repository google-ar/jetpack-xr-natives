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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_STYLE_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_STYLE_H_

#include "dear_imgui/imgui.h"

namespace imp::editor {

constexpr ImVec4 RGB24ToImVec4(ImU32 rgb) {
  return ImVec4(static_cast<float>((rgb >> 16) & 0xff) / 255.0f,
                static_cast<float>((rgb >> 8) & 0xff) / 255.0f,
                static_cast<float>(rgb & 0xff) / 255.0f, 1.0f);
}

constexpr ImVec4 RGBA32ToImVec4(ImU32 rgba) {
  return ImVec4(static_cast<float>((rgba >> 24) & 0xff) / 255.0f,
                static_cast<float>((rgba >> 16) & 0xff) / 255.0f,
                static_cast<float>((rgba >> 8) & 0xff) / 255.0f,
                static_cast<float>(rgba & 0xff) / 255.0f);
}

// Common to both dark and light palettes.
constexpr ImVec4 kCommonTransparent(RGBA32ToImVec4(0x00000000));

// Dark palette
constexpr ImVec4 kDarkSurface(RGB24ToImVec4(0x1B1B1B));
constexpr ImVec4 kDarkSurfaceAlt(RGB24ToImVec4(0x1E1F20));
constexpr ImVec4 kDarkSurfaceAlt2(RGB24ToImVec4(0x131314));
constexpr ImVec4 kDarkField(RGB24ToImVec4(0x282A2C));

constexpr ImVec4 kDarkTextGrey(RGB24ToImVec4(0xC5C7C5));
constexpr ImVec4 kDarkTextGreyAlt(RGB24ToImVec4(0x676767));
constexpr ImVec4 kDarkTextContast(RGB24ToImVec4(0xFFFFFF));

constexpr ImVec4 kDarkPrimary(RGB24ToImVec4(0x1D3774));
constexpr ImVec4 kDarkSecondary(RGB24ToImVec4(0xAEC6F6));
constexpr ImVec4 kDarkLowlight(RGB24ToImVec4(0x1D4974));
constexpr ImVec4 kDarkOutline(RGB24ToImVec4(0x676767));

constexpr ImVec4 kDarkSelectHighlight(RGB24ToImVec4(0x4C8DF6));
constexpr ImVec4 kDarkHover(RGB24ToImVec4(0x242425));
constexpr ImVec4 kDarkRed(RGB24ToImVec4(0xF55E57));
constexpr ImVec4 kDarkGreen(RGB24ToImVec4(0x44C265));
constexpr ImVec4 kDarkBlue(RGB24ToImVec4(0x4E8FF8));

// (TODO: (broken link)) Roll these legacy colors into standard palette
// The below colors are from the Google Material Design color palette.
// https://material.io/design/color/the-color-system.html
constexpr ImVec4 kBlue700(0.09803f, 0.4039f, 0.8235f, 1.0f);
constexpr ImVec4 kBlue900(0.0901f, 0.30588f, 0.6509f, 1.0f);
constexpr ImVec4 kGrey400(0.7411f, 0.7568f, 0.7764f, 1.0f);
constexpr ImVec4 kGrey500(0.6039f, 0.6274f, 0.6509f, 1.0f);
constexpr ImVec4 kGrey700(0.3725f, 0.3882f, 0.4078f, 1.0f);
constexpr ImVec4 kGrey900(0.1255f, 0.1294f, 0.1412f, 1.0f);

// Note: This doesn't use string_view for easy compatibility with ImGui APIs.
// Constant idiom from (broken link).
inline constexpr char kFloatFormat[] = "%.3f";

// (TODO: (broken link)) Try to remove use of alpha for UI colors.
ImVec4 WithAlpha(const ImVec4& color, float alpha);

void SetupEditorImGuiStyle();

void PushBaseIsfElementStyle();
void PopBaseIsfElementStyle();

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_STYLE_H_
