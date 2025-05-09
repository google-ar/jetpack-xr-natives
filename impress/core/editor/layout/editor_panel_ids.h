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

#include "absl/strings/string_view.h"

// The list of panels for the editor.
namespace imp::editor::panel_ids {

// Top-left panel that mainly houses the node graph, etc.
static constexpr absl::string_view kSceneWindow = "Scene";
// Top right panel that mainly houses component panels.
static constexpr absl::string_view kDetailsWindow = "Details";
// The big row of tabs at the bottom of the editor.
static constexpr absl::string_view kTabBar = "Tab Bar";
// The menu bar at the top of the editor.
static constexpr absl::string_view kMenuBar = "Menu Bar";
// The toolbar in the top center of the editor that houses play/pause, etc.
static constexpr absl::string_view kToolBar = "Toolbar";
static constexpr absl::string_view kFreeform = "";

}  // namespace imp::editor::panel_ids

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_LAYOUT_EDITOR_PANELS_H_
