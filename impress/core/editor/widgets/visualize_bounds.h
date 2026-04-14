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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VISUALIZE_BOUNDS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VISUALIZE_BOUNDS_H_

#include "absl/container/flat_hash_set.h"
#include "absl/strings/string_view.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Visualizes the bounds of the nodes within a glTF for the previewer.
class VisualizeBounds : public editor::Widget, public imp::Rememberer {
 public:
  explicit VisualizeBounds(BaseView& view);
  absl::string_view GetName() const override { return "##Visualize Bounds"; }
  void DrawImGui() override;

  // Draws the bounds for the relevant nodes based on the current mode.
  void DrawBounds();

 private:
  enum class Mode {
    // Draw the bounds only for the node selected in the hierarchy widget.
    // The bounds will be drawn in a green color.
    kShowSelectedBounds,
    // Draw the bounds for every node in the glTF.
    // Node's with a mesh will be drawn with a yellow color.
    // Node's with no mesh will be drawn with a white color.
    // The node selected in the hierarchy widget (if there is one) will be drawn
    // with a green color.
    kShowAllBounds
  };

  // Draws the bounds for all nodes in the scene.
  void DrawBoundsForAllNodes(const double3& camera_pos);
  // Recursively draws the bounds for a node and its children.
  void DrawBoundsForNodeRecursive(NodeHandle node, const double3& camera_pos);
  // Draws the bounds for a single node.
  void DrawBoundsForNode(NodeHandle node, const double3& camera_pos);

  bool HasCollider(NodeHandle node) const;

  BaseView& view_;
  Mode mode_ = Mode::kShowSelectedBounds;
  absl::flat_hash_set<NodeHandle> selected_nodes_;
  absl::flat_hash_set<NodeHandle> visited_nodes_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VISUALIZE_BOUNDS_H_
