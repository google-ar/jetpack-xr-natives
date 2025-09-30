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

// Determines where in the layout a widget should be drawn.
struct WidgetLayoutInfo {
  explicit WidgetLayoutInfo() : panel_id(PanelId::kFreeform) {}
  explicit WidgetLayoutInfo(PanelId panel_id) : panel_id(panel_id) {}

  PanelId panel_id;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGET_LAYOUT_INFO_H_
