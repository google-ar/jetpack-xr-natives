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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_UI_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_UI_HELPERS_H_

#include "dear_imgui/imgui.h"

namespace imp::editor {

constexpr ImVec2 kTableEntryImageSize(48, 48);

enum class IncludePadding { kNone, kCell };

void ImGuiCenterNextHorizontally(float next_width,
                                 IncludePadding include_padding);

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_UI_HELPERS_H_
