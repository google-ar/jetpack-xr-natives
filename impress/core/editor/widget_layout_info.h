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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGET_LAYOUT_INFO_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGET_LAYOUT_INFO_H_

#include "core/editor/layout/editor_panel_ids.h"

namespace imp::editor {

// Determines whether a widget will be removed when the layout is not 2D large
// screen.
// 2D large screen typically refers to the desktop monitor, it is applied to
// cases such as, desktop app, WASM and Remote Editor (through the browser on a
// desktop monitor)
enum class WidgetPresence {
  // The widget will be present in all layouts.
  kAlways,
  // The widget will only be present in 2D large screen layouts.
  kOnlyIn2DLargeScreen,
};

// In addition to the layout, this determines whether a widget will be visible
// or not.
enum class WidgetVisibility {
  kHidden,
  kVisible,
};

// Determines where in the layout a widget should be drawn.
struct WidgetLayoutInfo {
  explicit WidgetLayoutInfo() : panel_id(PanelId::kFreeform) {}
  WidgetLayoutInfo(PanelId panel_id,
                   WidgetPresence presence = WidgetPresence::kAlways,
                   WidgetVisibility visibility = WidgetVisibility::kVisible)
      : panel_id(panel_id), presence(presence), visibility(visibility) {}

  PanelId panel_id;
  WidgetPresence presence = WidgetPresence::kAlways;
  // Whether the widget will be visible by default, only works for widgets in
  // PanelId::kTabBar, PanelId::kLeftPanel, PanelId::kRightPanel,
  // PanelId::kFreeform.
  WidgetVisibility visibility = WidgetVisibility::kVisible;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGET_LAYOUT_INFO_H_
