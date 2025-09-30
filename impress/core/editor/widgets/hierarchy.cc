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

#include "absl/strings/str_cat.h"
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
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/node_handle.h"
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
        if (event.selected == active_node_) {
          return;
        }

        active_node_changed_ = true;
        active_node_ = event.selected;
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
    NodeHandle node = AcceptDragAndDropPayloadNode();
    if (node) {
      node->SetParentKeepWorldTransform(NodeHandle());
    }
    ImGui::EndDragDropTarget();
  }

  // Draw the filter.
  ImGui::Text("Filter:");
  ImGui::SameLine();
  filter_.Draw(
      GenerateUniqueImGuiLabel("filter", this, EditorControlFlags::kNone)
          .c_str());

  // Draw the Nodes section.
  view_.ForEachNode(
      [this](NodeHandle node) {
        std::optional<RobinSet<NodeHandle>> filtered_nodes = std::nullopt;
        if (filter_.IsActive()) {
          filtered_nodes.emplace();
          CollectFilteredNodes(filter_, *filtered_nodes, node);
        }
        DrawHierarchy(node, filtered_nodes);
      },
      NodeFlags::kIsRoot);
}

void Hierarchy::DrawHierarchy(
    NodeHandle node, std::optional<RobinSet<NodeHandle>> filtered_nodes) {
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

  if (!DrawNode(node, filtered_nodes)) {
    return;
  }

  if (node) {
    for (NodeHandle child : node->GetChildren()) {
      // Recurse with the children.
      DrawHierarchy(child, filtered_nodes);
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

bool Hierarchy::DrawNode(NodeHandle node,
                         std::optional<RobinSet<NodeHandle>> filtered_nodes) {
  ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                             ImGuiTreeNodeFlags_OpenOnDoubleClick |
                             ImGuiTreeNodeFlags_SpanAvailWidth;

  // Check if the node is currently selected.
  if (active_node_ == node) {
    flags |= ImGuiTreeNodeFlags_Selected;
    if (active_node_changed_) {
      ImGui::SetNextItemOpen(true, ImGuiCond_Always);
      active_node_changed_ = false;
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
    if (active_node_ == node) {
      // Deselect
      active_node_ = NodeHandle();
    } else {
      // Select
      active_node_ = node;
    }
    editor.SelectNode(active_node_);
  }

  if (MobileLongPress(GetTreeNodeLabelForNode(node))) {
    ImGui::OpenPopup(GetTreeNodeLabelForNode(node).c_str());
  }

  if (ImGui::BeginPopupContextItem(GetTreeNodeLabelForNode(node).c_str())) {
    if (ImGui::MenuItem("Add parent node")) {
      NodeHandle parent = view_.CreateNode();
      EditorTouch(parent);
      parent->SetParentKeepWorldTransform(node->GetParent());
      node->SetParentKeepWorldTransform(parent);
      editor.SelectNode(parent);
    }
    if (ImGui::MenuItem("Add child node")) {
      NodeHandle child = view_.CreateNode();
      EditorTouch(child);
      child->SetParentKeepWorldTransform(node);
      editor.SelectNode(child);
    }
    if (ImGui::MenuItem("Delete node")) {
      editor.SelectNode(NodeHandle());
      view_.DestroyNode(node);
    }
    if (HasValidMesh(node, CameraHelperOptions::kIncludeDescendants) &&
        ImGui::MenuItem("Focus on this node.")) {
      Editor& editor = view_.GetRegistry().Get<Editor>()->get();
      editor.GetDispatcher().Send(FocusOnSelectionEvent(node));
    }
    ImGui::EndPopup();
  }

  BeginDragAndDropSource(node);
  if (ImGui::BeginDragDropTarget()) {
    NodeHandle child = AcceptDragAndDropPayloadNode();
    if (child) {
      child->SetParentKeepWorldTransform(node);
    }
    ImGui::EndDragDropTarget();
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
