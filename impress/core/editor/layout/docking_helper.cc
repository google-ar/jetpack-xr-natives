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

#include "core/editor/layout/docking_helper.h"

#include <string>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "core/common/buffer_access.h"
#include "core/common/file_helpers.h"
#include "core/editor/editor_constants.h"
#include "core/editor/layout/editor_panel_ids.h"

namespace imp::editor {
// When docking is enabled, the bottom panel takes up 30% of the screen height.
constexpr float kDefaultBottomDockRatio = 0.3f;
// When docking is enabled, a side panel (left/right) takes up 20% of the screen
// width.
constexpr float kDefaultSideDockRatio = 0.25f;
constexpr absl::string_view kDockSpaceName = "ImpressEditorDockSpace";
constexpr absl::string_view kWindowLabelPrefix = "[Window][";
constexpr absl::string_view kWindowLabelSuffix = "]";

DockingHelper::DockingHelper() {
  Initialize();

  std::string filename = GetRepoDirectory() + std::string(kSavedLayoutIniFile);
  absl::StatusOr<BufferAccess> file = LoadFile(filename);

  // If the file exists, load the layout from the file.
  if (file.ok()) {
    initialized_with_saved_layout_ = true;
    ImGui::LoadIniSettingsFromMemory(
        reinterpret_cast<const char*>(file->Data()), file->Size());

    // Parse the window labels from the saved layout file. For example, the
    // content of the file may look like: [Window][Scene], and the window label
    // is "Scene".
    std::string entire_content(file->StringView());
    auto it = entire_content.find(kWindowLabelPrefix);
    while (it != std::string::npos) {
      auto end = entire_content.find(kWindowLabelSuffix,
                                     it + kWindowLabelPrefix.size());
      std::string window_label;
      window_label = entire_content.substr(
          it + kWindowLabelPrefix.size(), end - it - kWindowLabelPrefix.size());
      initial_visible_window_labels_.push_back(window_label);
      it = entire_content.find(kWindowLabelPrefix, end);
    }
  }
}

void DockingHelper::Initialize() {
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
  ImGui::DockBuilderSplitNode(
      /*space_to_split=*/remain_2, ImGuiDir_Left, kDefaultSideDockRatio,
      /*first_child=*/&dock_id_left_,
      /*the_remaining_space=*/&dock_id_center_);  // Split main dockspace

  // Dock windows into the created nodes
  ImGui::DockBuilderDockWindow(PanelIdToString(PanelId::kSceneWindow).c_str(),
                               dock_id_left_);
  ImGui::DockBuilderDockWindow(PanelIdToString(PanelId::kDetailsWindow).c_str(),
                               dock_id_right_);
  ImGui::DockBuilderDockWindow(PanelIdToString(PanelId::kViewport).c_str(),
                               dock_id_center_);

  ImGui::DockBuilderFinish(dockspace_id_);
}

ImGuiID DockingHelper::GetDockableSpaceId() { return dockspace_id_; }

ImGuiID DockingHelper::GetDockId(DockingType docking_type) {
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
