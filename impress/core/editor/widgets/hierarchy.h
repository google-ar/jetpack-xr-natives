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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_HIERARCHY_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_HIERARCHY_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/common/rememberer.h"
#include "core/common/robin_set.h"
#include "core/editor/widget.h"
#include "core/input/key_codes.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Shows a list of nodes in the GLTF/glb model.
class Hierarchy : public Widget, public imp::Rememberer {
 public:
  Hierarchy(BaseView& view, absl::string_view filter = "");
  // Do not show a header for this widget.
  absl::string_view GetName() const override { return "Nodes"; }
  void DrawImGui() override;
  ImGuiTreeNodeFlags GetTreeNodeFlags() const override;

  // Selects all nodes visible in the hierarchy between the last selected node
  // and the given node.
  void SelectRange(NodeHandle end_node);

 private:
  // Recursive function for printing out the nodes of the gltf renderer.
  void DrawHierarchy(NodeHandle node,
                     std::optional<RobinSet<NodeHandle>> filtered_nodes,
                     const absl::flat_hash_set<NodeHandle>& selected_nodes);
  // Print the Node as a ImGui tree node.
  bool DrawNode(NodeHandle node,
                std::optional<RobinSet<NodeHandle>> filtered_nodes,
                const absl::flat_hash_set<NodeHandle>& selected_nodes);
  // Gets the name of the node + ##<entityId> to display in the tree view.
  std::string GetTreeNodeLabelForNode(NodeHandle node);
  // Returns whether or not a long press happened on the given node.
  bool MobileLongPress(absl::string_view node);

  // Collects all nodes that are currently visible in the hierarchy.
  void CollectVisibleNodes(
      NodeHandle node,
      const std::optional<RobinSet<NodeHandle>>& filtered_nodes,
      std::vector<NodeHandle>& out_nodes);

  // Returns true if the node should be shown in the hierarchy (e.g. not an
  // internal editor node and passes the filter).
  bool ShouldShowNode(
      NodeHandle node,
      const std::optional<RobinSet<NodeHandle>>& filtered_nodes) const;

  // Returns true if the node is expanded in the hierarchy.
  bool IsNodeExpanded(
      NodeHandle node,
      const std::optional<RobinSet<NodeHandle>>& filtered_nodes) const;

  // Shows the popup menu for creating new nodes.
  void ShowCreateNodeMenu();

  BaseView& view_;
  absl::flat_hash_set<VirtualKeyCode> held_multi_select_keys_;
  absl::flat_hash_set<VirtualKeyCode> held_shift_keys_;
  bool selected_nodes_changed_ = false;
  float2 inspector_size_;
  float header_height_;
  ImGuiTextFilter filter_;
  RobinSet<NodeHandle> manually_expanded_nodes_;
  RobinSet<NodeHandle> manually_collapsed_nodes_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_HIERARCHY_H_
