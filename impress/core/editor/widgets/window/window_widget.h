/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_EDITOR_WIDGETS_WINDOW_WINDOW_WIDGET_H_
#define THIRD_PARTY_IMPRESS_EDITOR_WIDGETS_WINDOW_WINDOW_WIDGET_H_

#include "absl/strings/string_view.h"
#include "core/editor/editor.h"
#include "core/editor/widget.h"
#include "core/editor/widgets/window/window_configuration.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Shows a list of windows for the Impress editor, and allows users to change
// the visibility of those windows.
class WindowWidget : public editor::Widget {
 public:
  explicit WindowWidget(BaseView& view);
  void DrawImGui() override;
  absl::string_view GetName() const override { return "Window"; }

 private:
  void RestoreDefaultVisibility();

  BaseView& view_;
  editor::Editor& editor_;
  WindowConfiguration& window_configuration_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_EDITOR_WIDGETS_WINDOW_WINDOW_WIDGET_H_
