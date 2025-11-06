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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_WINDOW_WINDOW_CONFIGURATION_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_WINDOW_WINDOW_CONFIGURATION_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"

namespace imp::editor {

// Records the visibility of certain windows in the Impress editor.
class WindowConfiguration {
 public:
  enum WindowVisibility {
    kVisible,
    kHidden,
  };

  struct WindowInfo {
    WindowInfo(absl::string_view window_name,
               WindowVisibility initial_visibility)
        : window_name(window_name),
          initial_visibility(initial_visibility),
          current_visibility(initial_visibility) {}

    std::string window_name;
    bool initial_visibility;
    bool current_visibility;
  };
  using WindowInfoMap =
      absl::flat_hash_map<std::string, std::unique_ptr<WindowInfo>>;

  WindowConfiguration() {};

  // Adds a window to the configuration, so that its visibility can be
  // changed by the user.
  void AddWindow(absl::string_view window_name,
                 WindowVisibility initial_visibility);
  // Removes a window from the configuration, when the window is removed from
  // the editor.
  void RemoveWindow(absl::string_view window_name);

  // Returns names of all the windows, which is managed by the
  // WindowConfiguration.
  absl::Span<const std::string> GetWindowNames() const;

  // Returns the initial visibility of the window.
  bool GetWindowInitialVisibility(absl::string_view window_name) const;

  // Returns the current visibility of the window. If the window is not found
  // in the configuration, it is considered as visible.
  bool IsWindowVisible(absl::string_view window_name) const;

  // Sets the current visibility of the window.
  void SetWindowVisibility(absl::string_view window_name,
                           WindowVisibility visibility);

  // Restores the default visibility of the windows to the initial visibility.
  void RestoreDefaultVisibility();

 private:
  std::vector<std::string> window_names_;
  // Keep track of the window names and their initial and current visibilities.
  WindowInfoMap window_infos_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_WINDOW_WINDOW_CONFIGURATION_H_
