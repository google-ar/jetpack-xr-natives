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

ImVec4 WithAlpha(const ImVec4& color, float alpha);

constexpr ImVec4 kTransparent(0.0f, 0.0f, 0.0f, 0.0f);
constexpr ImVec4 kWhite(1.0f, 1.0f, 1.0f, 1.0f);
constexpr ImVec4 kBlack(0.0f, 0.0f, 0.0f, 1.0f);

// The below colors are from the Google Material Design color palette.
// https://material.io/design/color/the-color-system.html

constexpr ImVec4 kBlue100(0.8235f, 0.8901f, 0.9882f, 1.0f);
constexpr ImVec4 kBlue200(0.6823f, 0.796f, 0.9803f, 1.0f);
constexpr ImVec4 kBlue300(0.5411f, 0.7058f, 0.9725f, 1.0f);
constexpr ImVec4 kBlue400(0.4f, 0.6156f, 0.9647f, 1.0f);
constexpr ImVec4 kBlue500(0.2588f, 0.5215f, 0.9568f, 1.0f);
constexpr ImVec4 kBlue600(0.1019f, 0.4509f, 0.9098f, 1.0f);
constexpr ImVec4 kBlue700(0.09803f, 0.4039f, 0.8235f, 1.0f);
constexpr ImVec4 kBlue800(0.09411f, 0.3529f, 0.7372f, 1.0f);
constexpr ImVec4 kBlue900(0.0901f, 0.30588f, 0.6509f, 1.0f);

constexpr ImVec4 kGrey100(0.9451, 0.9529f, 0.9568f, 1.0f);
constexpr ImVec4 kGrey200(0.9098f, 0.91764f, 0.929f, 1.0f);
constexpr ImVec4 kGrey300(0.8549f, 0.8627f, 0.8784f, 1.0f);
constexpr ImVec4 kGrey400(0.7411f, 0.7568f, 0.7764f, 1.0f);
constexpr ImVec4 kGrey500(0.6039f, 0.6274f, 0.6509f, 1.0f);
constexpr ImVec4 kGrey600(0.5019f, 0.5254f, 0.5451f, 1.0f);
constexpr ImVec4 kGrey700(0.3725f, 0.3882f, 0.4078f, 1.0f);
constexpr ImVec4 kGrey800(0.2352f, 0.2509f, 0.2627f, 1.0f);
constexpr ImVec4 kGrey900(0.1255f, 0.1294f, 0.1412f, 1.0f);

constexpr ImVec4 kGreen300(0.50588f, 0.7882f, 0.149f, 1.0f);
constexpr ImVec4 kGreen500(0.2039f, 0.6588f, 0.3254f, 1.0f);

constexpr ImVec4 kRed300(0.949f, 0.5451f, 0.5098f, 1.0f);
constexpr ImVec4 kRed500(0.9176f, 0.2627f, 0.2078f, 1.0f);

constexpr ImVec4 kPurple900(0.4078f, 0.1137f, 0.6588f, 1.0f);

// Note: This doesn't use string_view for easy compatibility with ImGui APIs.
// Constant idiom from (broken link).
inline constexpr char kFloatFormat[] = "%.3f";

void SetupEditorImGuiStyle();

void PushBaseIsfElementStyle();
void PopBaseIsfElementStyle();

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_STYLE_H_
