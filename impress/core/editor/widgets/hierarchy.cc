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

#include "core/editor/widgets/hierarchy.h"

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"  // IWYU pragma: keep
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/registry.h"
#include "core/common/robin_set.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/editor_touch.h"
#include "core/editor/events.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/editor/layout/helpers.h"
#include "core/editor/ui/drag_and_drop_node.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/ncsb/scene_metadata.h"
#include "core/view/framework/camera/camera_helpers.h"

namespace imp::editor {
// The label to use in the UI for nodes that do not have a name set.
constexpr absl::string_view kUnamedNodeLabel = "<node>";

namespace {
static constexpr absl::string_view kNodesHeaderLabel = "Nodes";
#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(IOS)
// How long the user has to hold a menu item to open the context menu.
static constexpr absl::Duration kLongTapThresholdMs = absl::Milliseconds(500);
#endif

// Adds all ancestors of the given node to the set of filtered nodes.
void AddAllAncestors(RobinSet<NodeHandle>& filtered_nodes, NodeHandle node) {
  NodeHandle parent = node->GetParent();
  if (parent && !filtered_nodes.contains(parent)) {
    filtered_nodes.insert(parent);
    AddAllAncestors(filtered_nodes, parent);
  }
}

// Adds all nodes (and ancestors) whose name passes the given filter to the set.
void CollectFilteredNodes(const ImGuiTextFilter& filter,
                          RobinSet<NodeHandle>& filtered_nodes,
                          NodeHandle root) {
  if (filter.PassFilter(std::string(root->GetName()).c_str())) {
    filtered_nodes.insert(root);
    AddAllAncestors(filtered_nodes, root);
  }
  for (NodeHandle child : root->GetChildren()) {
    CollectFilteredNodes(filter, filtered_nodes, child);
  }
}

}  // namespace

Hierarchy::Hierarchy(BaseView& view, absl::string_view filter)
    : view_(view), filter_(std::string(filter).c_str()) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  editor.GetDispatcher().Connect(
      [this](const NodeSelectionChangedEvent& event) mutable {
        selected_nodes_changed_ = true;
      },
      this);

  editor.GetDispatcher().Connect(
      [this](const imp::KeyboardEvent& event) {
        if (HasKeyModifier(KeyModifier::CTRL_OR_GUI, event.key.modifiers)) {
          is_multi_selection_enabled_ =
              event.type == KeyboardEventType::kOnDown ? true : false;
          return;
        }

        is_multi_selection_enabled_ = false;
      },
      this);
}

ImGuiTreeNodeFlags Hierarchy::GetTreeNodeFlags() const {
  return ImGuiTreeNodeFlags_DefaultOpen;
}

void Hierarchy::DrawImGui() {
  // Decorate the widget header with a context menu trigger.
  if (MobileLongPress(kNodesHeaderLabel)) {
    ImGui::OpenPopup(kNodesHeaderLabel.data());
  }
  if (ImGui::BeginPopupContextItem(kNodesHeaderLabel.data())) {
    if (ImGui::MenuItem("New Node")) {
      NodeHandle node = view_.CreateNode();
      EditorTouch(node);
      Editor& editor = view_.GetRegistry().Get<Editor>()->get();
      editor.SelectNode(node);
    }
    ImGui::EndPopup();
  }
  if (ImGui::BeginDragDropTarget()) {
    std::vector<NodeHandle> nodes = AcceptDragAndDropPayloadNodes();
    for (NodeHandle& node : nodes) {
      if (node) {
        node->SetParentKeepWorldTransform(NodeHandle());
      }
    }
    ImGui::EndDragDropTarget();
  }

  // Draw the filter.
  ImGui::Text("Filter:");
  ImGui::SameLine();
  filter_.Draw(
      GenerateUniqueImGuiLabel("filter", this, EditorControlFlags::kNone)
          .c_str());

  const absl::flat_hash_set<NodeHandle>& selected_nodes =
      view_.GetRegistry().Get<Editor>()->get().GetSelectedNodes();
  // Draw the Nodes section.
  view_.ForEachNode(
      [this, &selected_nodes](NodeHandle node) {
        std::optional<RobinSet<NodeHandle>> filtered_nodes = std::nullopt;
        if (filter_.IsActive()) {
          filtered_nodes.emplace();
          CollectFilteredNodes(filter_, *filtered_nodes, node);
        }
        DrawHierarchy(node, filtered_nodes, selected_nodes);
      },
      NodeFlags::kIsRoot);
}

void Hierarchy::DrawHierarchy(
    NodeHandle node, std::optional<RobinSet<NodeHandle>> filtered_nodes,
    const absl::flat_hash_set<NodeHandle>& selected_nodes) {
  if (filtered_nodes.has_value() && !filtered_nodes->contains(node)) return;

  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  // Do not draw any of the editor nodes.
  if (node == editor.GetEditorRoot()) {
    return;
  }

#if IMP_RUNTIME(DEV)
  if (node->IsEditorStaging()) {
    return;
  }
#endif

  if (!DrawNode(node, filtered_nodes, selected_nodes)) {
    return;
  }

  if (node) {
    for (NodeHandle child : node->GetChildren()) {
      // Recurse with the children.
      DrawHierarchy(child, filtered_nodes, selected_nodes);
    }
  }
  ImGui::TreePop();
}

std::string Hierarchy::GetTreeNodeLabelForNode(NodeHandle node) {
  absl::string_view node_name = node->GetName();
  if (node_name.empty()) {
    node_name = kUnamedNodeLabel;
  }
  return absl::StrCat(node_name, "##", node->GetEntity().getId());
}

bool Hierarchy::DrawNode(
    NodeHandle node, std::optional<RobinSet<NodeHandle>> filtered_nodes,
    const absl::flat_hash_set<NodeHandle>& selected_nodes) {
  ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                             ImGuiTreeNodeFlags_OpenOnDoubleClick |
                             ImGuiTreeNodeFlags_SpanAvailWidth;
  bool is_node_selected = selected_nodes.contains(node);

  // Check if the node is currently selected.
  if (is_node_selected) {
    flags |= ImGuiTreeNodeFlags_Selected;
    if (selected_nodes_changed_) {
      ImGui::SetNextItemOpen(true, ImGuiCond_Always);
      selected_nodes_changed_ = false;
    }
  }

  // Check if the node has no children, in which case it is a leaf.
  if (node->GetChildren().empty()) {
    flags |= ImGuiTreeNodeFlags_Leaf;
  }

  // If the filter is active, control expanded state by a filter match.
  if (filtered_nodes.has_value()) {
    // Respect manually-collapsed nodes so the user can collapse parts of the
    // hierarchy even if they match the filter.
    ImGui::SetNextItemOpen(filtered_nodes.value().contains(node) &&
                               !manually_collapsed_nodes_.contains(node),
                           ImGuiCond_Always);
  } else {
    // If the filter is empty, set the expanded state to the set of expanded
    // nodes controlled by the user. This restores the hierarchy to its original
    // state if the user deletes the filter.
    manually_collapsed_nodes_.clear();
    ImGui::SetNextItemOpen(manually_expanded_nodes_.contains(node),
                           ImGuiCond_Always);
  }

  bool is_expanded =
      ImGui::TreeNodeEx(GetTreeNodeLabelForNode(node).c_str(), flags);

  // Update the manually-expanded and manually-collapsed sets accordingly.
  if (filtered_nodes.has_value()) {
    if (filtered_nodes.value().contains(node)) {
      if (!is_expanded) {
        manually_collapsed_nodes_.insert(node);
      } else {
        manually_collapsed_nodes_.erase(node);
      }
    }
  } else {
    if (is_expanded) {
      manually_expanded_nodes_.insert(node);
    } else {
      manually_expanded_nodes_.erase(node);
    }
  }

  Editor& editor = view_.GetRegistry().Get<Editor>()->get();

  // If the TreeNode was clicked to the right of the arrow, then toggle
  // selection.
  // TODO: don't accept click if dragging and dropping.
  bool is_mouse_beyond_arrow =
      node->GetChildren().empty() ||
      (ImGui::GetMousePos().x - ImGui::GetItemRectMin().x) >
          ImGui::GetTreeNodeToLabelSpacing();
  // Use IsMouseReleased && IsItemHovered instead of IsItemClicked becaues
  // IsItemClicked is called on mouse down, but we want the selection to occur
  // on mouse-up. This makes it possible to drag and drop nodes from the
  // hierarchy panel to the node_details panel, and is more consistent with the
  // rest of the UX.
  if (ImGui::IsMouseReleased(0) &&
      ImGui::IsItemHovered(ImGuiHoveredFlags_None) && is_mouse_beyond_arrow) {
    if (is_multi_selection_enabled_) {
      editor.SelectNode(node, Editor::SelectionMode::kMultipleNodes);
    } else {
      if (is_node_selected && selected_nodes.size() == 1) {
        // Deselect the node if the current node is the only selected node.
        // When we have multiple selected nodes and we click on one of them,
        // we want to deselect the other selected nodes and keep the current
        // node selected.
        editor.SelectNode(NodeHandle());
      } else {
        editor.SelectNode(node);
      }
    }
  }

  if (MobileLongPress(GetTreeNodeLabelForNode(node))) {
    ImGui::OpenPopup(GetTreeNodeLabelForNode(node).c_str());
  }

  bool has_multiple_selection = selected_nodes.size() > 1;
  if (ImGui::BeginPopupContextItem(GetTreeNodeLabelForNode(node).c_str())) {
    if (!has_multiple_selection && ImGui::MenuItem("Add parent node")) {
      NodeHandle parent = view_.CreateNode();
      EditorTouch(parent);
      parent->SetParentKeepWorldTransform(node->GetParent());
      node->SetParentKeepWorldTransform(parent);
      editor.SelectNode(parent);
    }
    if (!has_multiple_selection && ImGui::MenuItem("Add child node")) {
      NodeHandle child = view_.CreateNode();
      EditorTouch(child);
      child->SetParentKeepWorldTransform(node);
      editor.SelectNode(child);
    }
    if (ImGui::MenuItem(
            absl::StrFormat("Delete node%s", has_multiple_selection ? "s" : "")
                .c_str())) {
      // If the node is selected, delete all selected nodes. Otherwise, delete
      // the current node.
      std::vector<NodeHandle> nodes_to_delete =
          is_node_selected ? std::vector<NodeHandle>(selected_nodes.begin(),
                                                     selected_nodes.end())
                           : std::vector<NodeHandle>{node};

      editor.SelectNode(NodeHandle());
      for (auto& selected_node : nodes_to_delete) {
        view_.DestroyNode(selected_node);
      }
    }
    if (HasValidMesh(node, CameraHelperOptions::kIncludeDescendants) &&
        ImGui::MenuItem("Focus on this node.")) {
      Editor& editor = view_.GetRegistry().Get<Editor>()->get();
      editor.GetDispatcher().Send(FocusOnSelectionEvent());
    }
    ImGui::EndPopup();
  }

  if (is_node_selected) {
    // If the node is selected, start a drag-and-drop operation for all
    // currently selected nodes.
    BeginDragAndDropSource(
        std::vector<NodeHandle>(selected_nodes.begin(), selected_nodes.end()));
  } else {
    // Otherwise, start a drag-and-drop operation for the current node only.
    BeginDragAndDropSource(node);
  }

  // Peak at the drag & drop payload node to make sure this is a valid drop
  // target.
  std::vector<NodeHandle> peeked_drag_and_drop_payload_nodes =
      GetDragAndDropPayloadNodes();

  if (!peeked_drag_and_drop_payload_nodes.empty()) {
    bool is_valid_target = true;
    for (const NodeHandle& peeked_node : peeked_drag_and_drop_payload_nodes) {
      // The peeked node cannot be an ancestor of the target node, because it
      // would cause a cycle in the scene graph leading to a crash.
      bool is_ancestor_of_target =
          view_.GetPathManager().IsAncestorOf(peeked_node, node);

      // The peeked node cannot be reparented if it is a child of a base isf
      // file. That is because the Isf inheritance format is an additive merge
      // that doesn't support doing this, so it can't be saved out and loaded
      // back in.
      //
      // Outside sandbox mode, there will be no SceneMetadata which allows
      // arbitrary reparenting for debugging purposes.
      //
      // Note: This is intentionally checked here instead of when calling
      // BeginDragAndDropSource because these nodes can still be dragged into
      // other places (i.e. NodeSceneHandle, the AssetLibrary).
      auto peeked_metadata = peeked_node->GetComponent<SceneMetadata>();
      bool is_child_of_base =
          peeked_metadata && peeked_metadata->IsChildOfBase();

      if (is_ancestor_of_target || is_child_of_base) {
        is_valid_target = false;
        break;
      }
    }

    if (is_valid_target) {
      // It's a valid drop target.
      if (ImGui::BeginDragDropTarget()) {
        // Accept the drag & drop payload node, which clears the payload. Ensure
        // that the payload is the same as the one we peeked at earlier.
        std::vector<NodeHandle> drag_and_drop_payload_nodes =
            AcceptDragAndDropPayloadNodes();
        if (!drag_and_drop_payload_nodes.empty()) {
          

          // Reparent each node in the payload.
          for (auto& drag_and_drop_payload_node : drag_and_drop_payload_nodes) {
            drag_and_drop_payload_node->SetParentKeepWorldTransform(node);
          }
          // Expand the new parent node to show the dropped children.
          manually_expanded_nodes_.insert(node);
          manually_collapsed_nodes_.erase(node);
        }
        ImGui::EndDragDropTarget();
      }
    }
  }

  return is_expanded;
}

bool Hierarchy::MobileLongPress(absl::string_view node) {
#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(IOS)
  // On mobile try to open the context menu on long press
  if (!ImGui::IsPopupOpen(node.data()) && ImGui::IsItemActive()) {
    ImGuiContext& g = *GImGui;
    return g.ActiveIdTimer >= (float)absl::ToDoubleSeconds(kLongTapThresholdMs);
  }
#endif
  return false;
}

}  // namespace imp::editor
