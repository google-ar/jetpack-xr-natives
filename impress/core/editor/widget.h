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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGET_H_

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/invocable.h"

namespace imp::editor {

// Defines an IMGUI UI Widget interface.
struct Widget {
  // Returns the name of this Widget (mainly for consistently sorting Widgets).
  virtual absl::string_view GetName() const = 0;

  // Call ImGui API functions to draw the widget.
  virtual void DrawImGui() = 0;

  // Handle the x button click on the widget.
  virtual Invocable<void()> OnCloseButton() { return {}; }

  // Returns true if the widget has content to draw.
  virtual bool HasContent() const { return true; }

  virtual ImGuiTreeNodeFlags GetTreeNodeFlags() const {
    return ImGuiTreeNodeFlags_None;
  }

  virtual ~Widget() = default;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGET_H_
