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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VISUALIZE_COLLIDERS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VISUALIZE_COLLIDERS_H_

#include "absl/container/flat_hash_set.h"
#include "absl/strings/string_view.h"
#include "core/collision/collision_flags.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Visualizes the bounds of the nodes within a glTF for the previewer.
class VisualizeColliders : public editor::Widget, public imp::Rememberer {
 public:
  explicit VisualizeColliders(BaseView& view, bool use_view_dispatcher = false,
                              bool show_all_colliders = false);
  absl::string_view GetName() const override { return "##Visualize Colliders"; }
  void DrawImGui() override;

 private:
  enum class Mode {
    // Visualizes all colliders on the selected node and its descendants.
    kShowSelectedAndDescendantColliders,
    // Visualizes the colliders for every node in the scene.
    kShowAllColliders,
    // Visualizes only the colliders for the selected node.
    kShowSelectedNodeCollider,
  };

  // Draws the colliders for all relevant nodes based on the current mode.
  void DrawCollidersForAllNodes();
  // Recursively draws the colliders for a node and its children.
  void DrawCollidersForNodeRecursive(NodeHandle node,
                                     VisualizationStyle visualization_style);
  // Draws the colliders for a single node.
  void DrawCollidersForNode(NodeHandle node,
                            VisualizationStyle visualization_style);
  BaseView& view_;
  Mode mode_;
  absl::flat_hash_set<NodeHandle> selected_nodes_;
  absl::flat_hash_set<NodeHandle> visited_nodes_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VISUALIZE_COLLIDERS_H_
