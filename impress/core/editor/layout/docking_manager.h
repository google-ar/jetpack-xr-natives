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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_DOCKING_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_DOCKING_MANAGER_H_

#include "dear_imgui/imgui.h"

namespace imp::editor {

// Prepares for docking and stores the docking IDs.
class DockingManager {
 public:
  // Docking place of a panel.
  enum class DockingType {
    kLeft,
    kRight,
    kBottom,
  };

  // Creates the docking manager and initializes the docking layout.
  DockingManager();
  ImGuiID GetDockableSpaceId();
  // Returns the docking ID of the specified docking place.
  ImGuiID GetDockId(DockingType docking_type);

 private:
  ImGuiID dockspace_id_;
  ImGuiID dock_id_left_;
  ImGuiID dock_id_right_;
  ImGuiID dock_id_bottom_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_DOCKING_MANAGER_H_
