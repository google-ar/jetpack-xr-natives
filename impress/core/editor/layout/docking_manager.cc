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

#include "core/editor/layout/docking_manager.h"

#include <string>

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "core/editor/layout/editor_panel_ids.h"

namespace imp::editor {
// When docking is enabled, the bottom panel takes up 30% of the screen height.
constexpr float kDefaultBottomDockRatio = 0.3f;
// When docking is enabled, a side panel (left/right) takes up 20% of the screen
// width.
constexpr float kDefaultSideDockRatio = 0.2f;
constexpr absl::string_view kDockSpaceName = "ImpressEditorDockSpace";

DockingManager::DockingManager() {
  dockspace_id_ = ImGui::GetID(kDockSpaceName.data());

  ImGui::DockBuilderRemoveNode(dockspace_id_);  // Clear any previous layout
  ImGui::DockBuilderAddNode(dockspace_id_,
                            ImGuiDockNodeFlags_DockSpace);  // Add an empty node
  ImGui::DockBuilderSetNodeSize(dockspace_id_, ImGui::GetIO().DisplaySize);

  // The remaining space after splitting the right side.
  ImGuiID remain_1;
  ImGui::DockBuilderSplitNode(dockspace_id_, ImGuiDir_Right,
                              kDefaultSideDockRatio, &dock_id_right_,
                              &remain_1);  // Split main dockspace

  // The remaining space after splitting the bottom side.
  ImGuiID remain_2;
  ImGui::DockBuilderSplitNode(remain_1, ImGuiDir_Down, kDefaultBottomDockRatio,
                              &dock_id_bottom_,
                              &remain_2);  // Split main dockspace

  // The remaining space after splitting the left side.
  ImGuiID remain_3;
  ImGui::DockBuilderSplitNode(
      /*space_to_split=*/remain_2, ImGuiDir_Left, kDefaultSideDockRatio,
      /*first_child=*/&dock_id_left_,
      /*the_remaining_space=*/&remain_3);  // Split main dockspace

  // Dock windows into the created nodes
  ImGui::DockBuilderDockWindow(PanelIdToString(PanelId::kSceneWindow).c_str(),
                               dock_id_left_);
  ImGui::DockBuilderDockWindow(PanelIdToString(PanelId::kDetailsWindow).c_str(),
                               dock_id_right_);

  ImGui::DockBuilderFinish(dockspace_id_);
}

ImGuiID DockingManager::GetDockableSpaceId() { return dockspace_id_; }

ImGuiID DockingManager::GetDockId(DockingType docking_type) {
  switch (docking_type) {
    case DockingType::kLeft:
      return dock_id_left_;
    case DockingType::kRight:
      return dock_id_right_;
    case DockingType::kBottom:
      return dock_id_bottom_;
  }
}

}  // namespace imp::editor
