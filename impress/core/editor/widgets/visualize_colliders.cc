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

#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "core/collision/collision_flags.h"
#include "core/common/registry.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/collision/collision_manager.h"

namespace imp::editor {

VisualizeColliders::VisualizeColliders(BaseView& view, bool use_view_dispatcher,
                                       bool show_all_colliders)
    : view_(view) {
  mode_ = show_all_colliders ? Mode::kShowAllColliders
                             : Mode::kShowSelectedAndDescendantColliders;

  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  selected_nodes_ = editor.GetSelectedNodes();
  Dispatcher& dispatcher =
      use_view_dispatcher ? view_.GetDispatcher() : editor.GetDispatcher();

  dispatcher.Connect(
      [this](const NodeSelectionChangedEvent&) {
        selected_nodes_ =
            view_.GetRegistry().Get<Editor>()->get().GetSelectedNodes();
      },
      this);

  dispatcher.Connect(
      [this](const EditorSettingChangedEvent& event) {
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
  visited_nodes_.clear();

  switch (mode_) {
    case Mode::kShowSelectedAndDescendantColliders: {
      for (const NodeHandle& node : selected_nodes_) {
        DrawCollidersForNodeRecursive(node, VisualizationStyle::kSelected);
      }
      break;
    }
    case Mode::kShowAllColliders: {
      // Draw all colliders for all nodes.
      DrawCollidersForAllNodes();
      break;
    }
    case Mode::kShowSelectedNodeCollider:
      for (const NodeHandle& node : selected_nodes_) {
        DrawCollidersForNode(node, VisualizationStyle::kSelected);
      }
      break;
  }
}

void VisualizeColliders::DrawCollidersForNodeRecursive(
    const NodeHandle node, const VisualizationStyle visualization_style) {
  if (!node || visited_nodes_.contains(node)) return;

  DrawCollidersForNode(node, visualization_style);

  VisualizationStyle child_viz_style =
      (selected_nodes_.contains(node) ? VisualizationStyle::kSelectedDescendent
                                      : visualization_style);
  for (const NodeHandle& child : node->GetChildren()) {
    DrawCollidersForNodeRecursive(child, child_viz_style);
  }
}

void VisualizeColliders::DrawCollidersForAllNodes() {
  view_.ForEachNode(
      [this](NodeHandle node) {
        if (!node) return;

        DrawCollidersForNodeRecursive(node, VisualizationStyle::kNotSelected);
      },
      NodeFlags::kIsRoot);
}

void VisualizeColliders::DrawCollidersForNode(
    NodeHandle node, VisualizationStyle visualization_style) {
#if IMP_RUNTIME(DEV)
  if (!node) return;

  Editor& editor = view_.GetRegistry().Get<Editor>()->get();

  if (node == editor.GetEditorRoot()) return;

  // If we've already visited this node, don't draw it again.
  if (!visited_nodes_.insert(node).second) return;

  VisualizationStyle node_viz_style =
      (selected_nodes_.contains(node) ? VisualizationStyle::kSelected
                                      : visualization_style);

  view_.GetCollisionManager().VisualizeCollidersForNode(node, node_viz_style);

#endif
}

}  // namespace imp::editor
