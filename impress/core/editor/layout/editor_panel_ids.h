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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_EDITOR_PANELS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_EDITOR_PANELS_H_

#include <string>

// The list of panels for the editor.
namespace imp::editor {

// Indicate the location of the widget in the editor.
enum class PanelId {
  // The Scene panel mainly contains the 3D scene graph / node hierarchy.
  kSceneWindow,
  // The Details panel contains the node details and component editor.
  kDetailsWindow,
  // The Viewport panel contains the rendered scene.
  kViewport,
  // The left panel contains the widgets that are pinned to the left side of the
  // screen, along with the scene window.
  kLeftPanel,
  // The right panel contains the widgets that are pinned to the right side of
  // the screen, along with the details window.
  kRightPanel,
  // The menu bar contains the widgets that are pinned to the top of the screen.
  kMenuBar,
  // The tool bar is a panel in the top-middle of the screen, that houses
  // play/pause, etc.
  kToolBar,
  // The tab bar contains the widgets that are pinned to the bottom of the
  // screen.
  kTabBar,
  kFreeform,
};

inline std::string PanelIdToString(PanelId panel_id) {
  switch (panel_id) {
    case PanelId::kSceneWindow:
      return "Scene";
    case PanelId::kDetailsWindow:
      return "Details";
    case PanelId::kViewport:
      return "Viewport";
    case PanelId::kLeftPanel:
      return "Left Panel";
    case PanelId::kRightPanel:
      return "Right Panel";
    case PanelId::kMenuBar:
      return "Menu Bar";
    case PanelId::kToolBar:
      return "Tool Bar";
    case PanelId::kTabBar:
      return "Tab Bar";
    case PanelId::kFreeform:
      return "Freeform";
  }
  return "Unknown";
}

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_EDITOR_PANELS_H_
