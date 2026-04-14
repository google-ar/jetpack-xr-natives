// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_DEBUG_DRAW_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_DEBUG_DRAW_WIDGET_H_

#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/editor/widget.h"

namespace imp::editor {

// Widget showing which debug draw tags are enabled, and allows the user to
// toggle them on/off.
class DebugDrawWidget : public Widget {
 public:
  absl::string_view GetName() const override { return "Debug Draw"; }
  void DrawImGui() override;

 private:
  void SortTags();

  bool sort_ascending_ = true;
  std::vector<std::string> sorted_tags_;
  ImGuiTextFilter filter_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_DEBUG_DRAW_WIDGET_H_
