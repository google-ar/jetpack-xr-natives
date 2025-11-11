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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_DOCKING_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_DOCKING_HELPER_H_

#include <string>
#include <vector>

#include "absl/types/span.h"
#include "dear_imgui/imgui.h"

namespace imp::editor {

// Prepares for docking and stores the docking IDs.
class DockingHelper {
 public:
  // Docking place of a panel.
  enum class DockingType {
    kLeft,
    kRight,
    kBottom,
  };

  // Creates the docking helper and initializes the docking layout.
  DockingHelper();

  // Initializes the docking layout from scratch.
  void Initialize();

  // Returns the docking space ID.
  ImGuiID GetDockableSpaceId();

  // Returns the docking ID of the specified docking place.
  ImGuiID GetDockId(DockingType docking_type);

  // Returns true if the docking layout is initialized with a saved layout.
  bool IsInitializedWithSavedLayout() const {
    return initialized_with_saved_layout_;
  }

  // Returns the initial visible window labels.
  absl::Span<const std::string> GetInitialVisibleWindowLabels() const {
    return absl::MakeSpan(initial_visible_window_labels_);
  }

 private:
  ImGuiID dockspace_id_;
  ImGuiID dock_id_left_;
  ImGuiID dock_id_right_;
  ImGuiID dock_id_bottom_;
  bool initialized_with_saved_layout_ = false;
  std::vector<std::string> initial_visible_window_labels_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_DOCKING_HELPER_H_
