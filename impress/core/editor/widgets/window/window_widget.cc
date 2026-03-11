// Copyright 2025 Google LLC
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

#include "core/editor/widgets/window/window_widget.h"

#include <string>

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/widgets/window/window_configuration.h"
#include "core/view/base_view.h"

namespace imp::editor {
namespace {
// TODO: change the text to "Restore Default Layout" after
// renaming the function RestoreDefault to RestoreDefaultLayout.
constexpr absl::string_view kRestoreDefaultText = "Restore Default Visibility";
constexpr absl::string_view kHideAllText = "Hide All";
constexpr absl::string_view kShowAllText = "Show All";
}  // namespace

WindowWidget::WindowWidget(BaseView& view)
    : view_(view),
      editor_(*view_.GetRegistry().Get<Editor>()),
      window_configuration_(*view_.GetRegistry().Get<WindowConfiguration>()) {}

void WindowWidget::DrawImGui() {
  bool are_all_windows_using_initial_visibility = true;
  for (const auto& window_name : window_configuration_.GetWindowNames()) {
    bool show_window = window_configuration_.IsWindowVisible(window_name);
    if (ImGui::MenuItem(window_name.data(), nullptr, &show_window)) {
      window_configuration_.SetWindowVisibility(
          window_name, show_window
                           ? WindowConfiguration::WindowVisibility::kVisible
                           : WindowConfiguration::WindowVisibility::kHidden);
    }
    are_all_windows_using_initial_visibility &=
        window_configuration_.GetWindowInitialVisibility(window_name) ==
        show_window;
  }

  ImGui::Separator();
  bool restore_default_visibility = false;
  if (ImGui::MenuItem(std::string(kRestoreDefaultText).c_str(), nullptr,
                      &restore_default_visibility)) {
    window_configuration_.RestoreDefault();
  }

  const absl::string_view text = hide_all_ ? kShowAllText : kHideAllText;
  if (ImGui::MenuItem(std::string(text).c_str())) {
    hide_all_ = !hide_all_;
  }
  window_configuration_.SetHideAllWindows(hide_all_);

#if !IMP_PLATFORM(WASM)
  ImGui::Separator();
  bool save_layout = false;
  if (ImGui::MenuItem(std::string("Save Layout").c_str(), nullptr,
                      &save_layout)) {
    window_configuration_.SaveLayoutToIniFile();
  }
#endif
}

void WindowWidget::RestoreDefault() { window_configuration_.RestoreDefault(); }
}  // namespace imp::editor
