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

#include "core/editor/widgets/visualize_origins.h"

#include "core/common/debug_draw.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/camera/camera_manager.h"

namespace imp::editor {

constexpr debug_draw::DebugColor kOriginColor = debug_draw::DebugColor::kPink;
constexpr float kCrossHalfExtent = 0.05f;

VisualizeOrigins::VisualizeOrigins(BaseView& view) : view_(view) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  Dispatcher& editor_dispatcher = editor.GetDispatcher();
  editor_dispatcher.Connect(
      [this](const NodeSelectionChangedEvent& event) mutable {
        // We only support single selection for the visualize origins widget.
        // Track which node is assigned in the hierarchy widget.
        selected_node_ =
            view_.GetRegistry().Get<Editor>()->get().GetSingleSelectedNode();
      },
      this);
  editor_dispatcher.Connect(
      [this](const EditorSettingChangedEvent& event) mutable {
        // Switch modes based on the "show all origins" setting.
        if (event.show_all_origins_enabled.has_value()) {
          if (*event.show_all_origins_enabled) {
            mode_ = Mode::kShowAllOrigins;
          } else {
            mode_ = Mode::kShowSelectedOrigins;
          }
        }
      },
      this);
}

void VisualizeOrigins::DrawImGui() {
  switch (mode_) {
    case Mode::kShowSelectedOrigins: {
      // Only draw the selected node, if there is one.
      if (selected_node_) {
        DrawOriginForNode(selected_node_);
      }
      break;
    }
    case Mode::kShowAllOrigins: {
      // Draw all origins for all nodes.
      DrawOriginsForAllNodes();
      break;
    }
  }
}

void VisualizeOrigins::DrawOriginForNode(NodeHandle node) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  ComponentHandle<CameraComponent> editor_camera = editor.GetCamera();
  float distance;

  if (view_.IsPreciseTranslationEnabled()) {
    distance = (float)norm(node->GetWorldPositionPrecise() -
                           editor_camera->GetNode()->GetWorldPositionPrecise());
  } else {
    distance = norm(node->GetWorldPosition() -
                    editor_camera->GetNode()->GetWorldPosition());
  }

  float size = kCrossHalfExtent * distance;

  auto color = debug_draw::GetColor(kOriginColor);
  debug_draw::Local(node->GetEntity())
      .Line(float3(-size, 0.0f, 0.0f), float3(size, 0.0f, 0.0f), color);
  debug_draw::Local(node->GetEntity())
      .Line(float3(0.0f, -size, 0.0f), float3(0.0f, size, 0.0f), color);
  debug_draw::Local(node->GetEntity())
      .Line(float3(0.0f, 0.0f, -size), float3(0.0f, 0.0f, size), color);
}

void VisualizeOrigins::DrawOriginsForNodeRecursive(NodeHandle node) {
  // If this node is part of the editor, return early.
  // Also, make sure that its children are also treated as part of the editor
  // and not shown.
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  if (node == editor.GetEditorRoot()) {
    return;
  }

  DrawOriginForNode(node);

  for (NodeHandle child : node->GetChildren()) {
    DrawOriginsForNodeRecursive(child);
  }
}

void VisualizeOrigins::DrawOriginsForAllNodes() {
  view_.ForEachNode(
      [this](NodeHandle node) {
        if (node == view_.GetCameraManager().GetCamera()->GetNode()) {
          return;
        }

        DrawOriginsForNodeRecursive(node);
      },
      NodeFlags::kIsRoot);
}

}  // namespace imp::editor
