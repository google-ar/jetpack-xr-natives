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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_HELPERS_H_
#include <string>

#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/bit_flag.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/math/math.h"
#include "core/math/vec.h"

namespace imp::editor {

// Helper function for generating unique ids for editor fields.
template <typename T>
inline std::string GenerateUniqueImGuiLabel(
    absl::string_view field_name, T* field,
    EditorControlFlags editor_control_flags = EditorControlFlags::kDefault) {
  if (CheckBit(editor_control_flags, EditorControlFlags::kDisplayLabel)) {
    return absl::StrFormat("%s##%p_%s", field_name, field, field_name);
  } else {
    return absl::StrFormat("##%p_%s", field, field_name);
  }
}

// Calculates Dear ImGui button size. See
// https://github.com/ocornut/imgui/issues/3714 for explanation.
inline ImVec2 CalculateButtonSize(std::string label) {
  ImGuiStyle& style = ImGui::GetStyle();
  ImVec2 size = ImGui::CalcTextSize(label.c_str());
  size.x += style.ItemInnerSpacing.x + style.FramePadding.x * 2.0f;
  size.y += style.ItemInnerSpacing.y + style.FramePadding.y * 2.0f;
  return size;
}

// Helper for linear interpolation of an ImVec2.
inline ImVec2 LerpImVec2(ImVec2 from, ImVec2 to, float kLerpFactor) {
  float2 lerped_value =
      lerp(float2(from.x, from.y), float2(to.x, to.y), kLerpFactor);
  return {lerped_value.x, lerped_value.y};
}

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_HELPERS_H_
