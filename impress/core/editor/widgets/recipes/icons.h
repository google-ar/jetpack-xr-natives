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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_ICONS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_ICONS_H_

#define IMGUI_DEFINE_MATH_OPERATORS
#include "dear_imgui/imgui.h"

namespace imp::editor::recipe_internal {

enum class IconType : ImU32 {
  Flow,
  Circle,
  Square,
  Grid,
  RoundSquare,
  Diamond
};

void DrawIcon(ImDrawList* drawList, const ImVec2& a, const ImVec2& b,
              IconType type, bool filled, ImU32 color, ImU32 innerColor);

}  // namespace imp::editor::recipe_internal

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_RECIPES_ICONS_H_
