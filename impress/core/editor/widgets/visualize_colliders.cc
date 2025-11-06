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

#include "core/editor/widgets/visualize_colliders.h"

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "core/collision/collision_flags.h"
#include "core/common/platform_helpers.h"
#include "core/common/registry.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/collision/collision_manager.h"

namespace imp::editor {

VisualizeColliders::VisualizeColliders(BaseView& view, bool use_view_dispatcher,
                                       bool show_all_colliders)
    : view_(view) {
  mode_ = show_all_colliders ? Mode::kShowAllColliders
                             : Mode::kShowSelectedAndDescendantColliders;
  Dispatcher& dispatcher =
      use_view_dispatcher
          ? view_.GetDispatcher()
          : view_.GetRegistry().Get<Editor>()->get().GetDispatcher();
  dispatcher.Connect(
      [this](const NodeSelectionChangedEvent& event) mutable {
        // We only support single selection for the visualize colliders widget.
        // Track which node is assigned in the hierarchy widget.
        selected_node_ =
            view_.GetRegistry().Get<Editor>()->get().GetSingleSelectedNode();
      },
      this);
  dispatcher.Connect(
      [this](const EditorSettingChangedEvent& event) mutable {
        // Switch modes based on the "show all colliders" setting.
        if (event.show_all_colliders_enabled.has_value()) {
          if (*event.show_all_colliders_enabled) {
            IMP_LOG(imp::INFO) << "Visualizing all colliders";
            mode_ = Mode::kShowAllColliders;
          } else {
            IMP_LOG(imp::INFO) << "Visualizing selected colliders";
            mode_ = Mode::kShowSelectedAndDescendantColliders;
          }
        }
      },
      this);
}

void VisualizeColliders::DrawImGui() {
  switch (mode_) {
    case Mode::kShowSelectedAndDescendantColliders: {
      if (selected_node_) {
        DrawCollidersForNodeRecursive(selected_node_,
                                      VisualizationStyle::kSelected);
      }
      break;
    }
    case Mode::kShowAllColliders: {
      // Draw all bounds for all nodes.
      DrawCollidersForAllNodes();
      break;
    }
    case Mode::kShowSelectedNodeCollider:
      if (selected_node_) {
        DrawCollidersForNode(selected_node_, VisualizationStyle::kSelected);
      }
      break;
  }
}

void VisualizeColliders::DrawCollidersForNodeRecursive(
    NodeHandle node, VisualizationStyle visualization_style) {
  DrawCollidersForNode(node, visualization_style);

  VisualizationStyle child_viz_style =
      (node == selected_node_ ? VisualizationStyle::kSelectedDescendent
                              : visualization_style);
  for (NodeHandle child : node->GetChildren()) {
    DrawCollidersForNodeRecursive(child, child_viz_style);
  }
}

void VisualizeColliders::DrawCollidersForAllNodes() {
  view_.ForEachNode(
      [this](NodeHandle node) {
        if (node == view_.GetCameraManager().GetCamera()->GetNode()) {
          return;
        }

        DrawCollidersForNodeRecursive(node, VisualizationStyle::kNotSelected);
      },
      NodeFlags::kIsRoot);
}

void VisualizeColliders::DrawCollidersForNode(
    NodeHandle node, VisualizationStyle visualization_style) {
#if IMP_RUNTIME(DEV)
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  if (node == editor.GetEditorRoot()) {
    return;
  }

  VisualizationStyle node_viz_style =
      (node == selected_node_ ? VisualizationStyle::kSelected
                              : visualization_style);

  view_.GetCollisionManager().VisualizeCollidersForNode(node, node_viz_style);

#endif
}

}  // namespace imp::editor
