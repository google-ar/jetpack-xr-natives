// Copyright 2024 Google LLC
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

#include "core/view/platforms/xr_android/xr_editor_plugin.h"

#include <memory>

#include "core/editor/editor_constants.h"
#include "core/editor/editor_plugin.h"
#include "core/editor/events.h"
#include "core/editor/layout/layout_composer.h"
#include "core/editor/layout/layout_config.proto.imp.h"
#include "core/editor/xr/xr_editor_ui.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

void XrEditorPlugin::OnEditorInitialized() {
  editor_node_ = view_->CreateNode();
  editor_node_->SetEnabled(false);
  editor_node_->SetName("Impress Editor Node");

  view_->GetDispatcher().Connect(
      [this](const EditorEnabledEvent& event) {
        if (event.enabled) {
          editor_node_->GetOrAddComponent<XrEditorUi>();
        }
        view_->GetHost()->TryGetExtension()->SetEnabled(event.enabled);
        editor_node_->SetEnabled(event.enabled);
      },
      view_);
}

std::unique_ptr<LayoutComposer> XrEditorPlugin::CreateLayoutComposer() {
  imp::editor::LayoutConfig layout_config =
      imp::editor::kDefaultMobileLayoutConfig;
  // Render a cursor in XR.
  layout_config.show_cursor = true;
  // Pin the window to the top.
  layout_config.initial_tabbed_window_state.pin_state =
      LayoutConfig::PINNED_TO_TOP;
  layout_config.initial_tabbed_window_state.max_window_height_multiplier = 1;
  return std::make_unique<LayoutComposer>(layout_config);
}

EditorPlugin::CameraConfiguration XrEditorPlugin::GetCameraConfiguration()
    const {
  return EditorPlugin::CameraConfiguration::kAppCameraOnly;
}

}  // namespace imp::editor
