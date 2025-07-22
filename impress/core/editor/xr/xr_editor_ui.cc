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

#include "core/editor/xr/xr_editor_ui.h"

#include "core/editor/components/world_space_editor_ui.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {
void XrEditorUi::Setup(const SetupOptions& options) {
  // Create a world-space Editor.
  editor_node_ = GetNode()->CreateChildNode();
  editor_node_
      ->AddComponent<imp::editor::WorldSpaceEditorUi>(
          options.editor_resolution, options.editor_panel_settings)
      .Then([this, editor_scale = options.editor_scale](
                imp::ComponentHandle<imp::editor::WorldSpaceEditorUi>
                    world_space_editor_ui) {
        world_space_editor_ui_ = world_space_editor_ui;
        editor_node_->SetLocalScale(editor_scale);
      })
      .KeptBy(this);
}

}  // namespace imp::editor
