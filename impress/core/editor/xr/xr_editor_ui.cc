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

#include "core/common/log.h"
#include "core/editor/components/world_space_editor_ui.h"
#include "core/editor/xr/xr_grab_handle.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp::editor {
void XrEditorUi::Setup(const SetupOptions& options) {
#if !IMP_RUNTIME(DEV)
  IMP_LOG(imp::FATAL) << "The XrEditorUi requires --define=IMP_DEV_RUNTIME=1.";
#endif
  imp::NodeHandle grab_handle_node = GetView().CreateNode();
  grab_handle_node->SetName("Grab Handle");
  grab_handle_node->SetParent(GetNode());
  xr_editor_grab_handle_ = grab_handle_node->AddComponent<XrGrabHandle>(
      /*radius=*/options.distance_from_camera,
      /*initial_horizontal_offset_degrees=*/options.horizontal_offset_degrees,
      /*initial_vertical_offset_degrees=*/options.vertical_offset_degrees);

  // Create a world-space Editor.
  editor_node_ = GetView().CreateNode();
  editor_node_->SetName("World Space Editor UI");
  editor_node_
      ->AddComponent<imp::editor::WorldSpaceEditorUi>(options.editor_resolution)
      .Then([this, grab_handle_node, editor_scale = options.editor_scale](
                imp::ComponentHandle<imp::editor::WorldSpaceEditorUi>
                    world_space_editor_ui) {
        world_space_editor_ui_ = world_space_editor_ui;
        editor_node_->SetLocalScale(editor_scale);
        editor_node_->SetParent(grab_handle_node);
        // Position the Editor panel just below the grab handle.
        editor_node_->SetLocalPosition({0.0f, -1.1f, 0.0f});
      })
      .KeptBy(this);
}

}  // namespace imp::editor
