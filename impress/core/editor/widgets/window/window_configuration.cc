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

#include "core/editor/widgets/window/window_configuration.h"

#include <algorithm>
#include <memory>
#include <string>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"

namespace imp::editor {

void WindowConfiguration::AddWindow(absl::string_view window_name,
                                    WindowVisibility initial_visibility) {
  if (window_infos_.contains(window_name)) {
    IMP_LOG(imp::ERROR) << "Window(Widget) with name " << window_name
               << " already exists. This may messes up the UI. Please use a "
                  "unique name.";
    return;
  }
  window_names_.push_back(std::string(window_name));
  window_infos_[window_name] =
      std::make_unique<WindowInfo>(window_name, initial_visibility);
}

void WindowConfiguration::RemoveWindow(absl::string_view window_name) {
  if (auto iter = window_infos_.find(window_name);
      iter != window_infos_.end()) {
    window_infos_.erase(iter);
    auto name_iter =
        std::find(window_names_.begin(), window_names_.end(), window_name);
    window_names_.erase(name_iter);
  }
}

absl::Span<const std::string> WindowConfiguration::GetWindowNames() const {
  return absl::MakeConstSpan(window_names_);
}

bool WindowConfiguration::GetWindowInitialVisibility(
    absl::string_view window_name) const {
  if (auto itr = window_infos_.find(window_name); itr != window_infos_.end()) {
    return itr->second->initial_visibility == WindowVisibility::kVisible;
  }
  return true;
}

bool WindowConfiguration::IsWindowVisible(absl::string_view window_name) const {
  if (auto itr = window_infos_.find(window_name); itr != window_infos_.end()) {
    return itr->second->current_visibility == WindowVisibility::kVisible;
  }
  return true;
}

void WindowConfiguration::SetWindowVisibility(absl::string_view window_name,
                                              WindowVisibility visibility) {
  if (auto itr = window_infos_.find(window_name); itr != window_infos_.end()) {
    itr->second->current_visibility = visibility;
  }
}

void WindowConfiguration::RestoreDefault() {
  for (auto iter = window_names_.begin(); iter != window_names_.end(); ++iter) {
    WindowInfo* info = window_infos_[*iter].get();
    info->current_visibility = info->initial_visibility;
  }
  should_restore_default_layout_ = true;
}

void WindowConfiguration::SetHideAllWindows(bool hide_all) {
  hide_all_ = hide_all;
}

void WindowConfiguration::SaveLayoutToIniFile() {
  should_save_layout_to_ini_file_ = true;
}

bool WindowConfiguration::ShouldSaveLayoutToIniFile() const {
  return should_save_layout_to_ini_file_;
}

void WindowConfiguration::NotifyLayoutSaved() {
  should_save_layout_to_ini_file_ = false;
}

}  // namespace imp::editor
