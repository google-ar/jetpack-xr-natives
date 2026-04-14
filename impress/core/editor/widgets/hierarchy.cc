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

#include <cstdlib>
#include <iterator>
#include <optional>
#include <string>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"  // IWYU pragma: keep
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/common/robin_set.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/editor_clipboard.h"
#include "core/editor/editor_info.h"
#include "core/editor/editor_style.h"
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
#include "core/render/primitive_shape_renderer.h"
#include "core/render/primitive_shape_type.h"
#include "core/view/framework/camera/camera_helpers.h"

namespace imp::editor {
// The label to use in the UI for nodes that do not have a name set.
constexpr absl::string_view kUnamedNodeLabel = "<node>";

namespace {
static constexpr absl::string_view kNodesHeaderLabel = "Nodes";

constexpr VirtualKeyCode kMultiSelectKeyCodes[] = {
    VirtualKeyCode::VK_LEFT_SUPER,
    VirtualKeyCode::VK_RIGHT_SUPER,
    VirtualKeyCode::VK_LEFT_CTRL,
    VirtualKeyCode::VK_RIGHT_CTRL,
};

// Keycodes used for shift-click selecting a range of nodes.
constexpr VirtualKeyCode kShiftKeyCodes[] = {
    VirtualKeyCode::VK_LEFT_SHIFT,
    VirtualKeyCode::VK_RIGHT_SHIFT,
};

#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(IOS)
// How long the user has to hold a menu item to open the context menu.
static constexpr absl::Duration kLongTapThresholdMs = absl::Milliseconds(500);
#endif

std::string GetPasteLabel(const EditorClipboard& editor_clipboard) {
  const std::vector<absl::string_view>& clipboard_node_names =
      editor_clipboard.GetClipboardNodeNames();
  std::string paste_label_suffix;
  if (clipboard_node_names.size() == 1) {
    if (clipboard_node_names.front().empty()) {
      paste_label_suffix = kUnamedNodeLabel;
    } else {
      paste_label_suffix =
          absl::StrCat("\"", clipboard_node_names.front(), "\"");
    }
  } else {
    paste_label_suffix = absl::StrCat(clipboard_node_names.size(), " nodes");
  }
  return absl::StrCat("Paste ", paste_label_suffix);
}

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

NodeHandle CreateAndSelectEmptyNode(BaseView& view) {
  NodeHandle node = view.CreateNode();
  EditorTouch(node);
  Editor& editor = view.GetRegistry().Get<Editor>()->get();
  editor.SelectNode(node);
  return node;
}

void CreateAndSelectPrimitiveShapeNode(BaseView& view,
                                       PrimitiveShapeType shape_type) {
  Future<NodeHandle> node =
      PrimitiveShapeRenderer::CreatePrimitive(view, shape_type);

  node.Then([&view](NodeHandle node) {
        EditorTouch(node);
        Editor& editor = view.GetRegistry().Get<Editor>()->get();
        editor.SelectNode(node);
      })
      .KeptBy(&view);
}

}  // namespace

Hierarchy::Hierarchy(BaseView& view, absl::string_view filter)
    : view_(view), filter_(std::string(filter).c_str()) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  editor.GetDispatcher().Connect(
      [this, &editor](const NodeSelectionChangedEvent& event) mutable {
        const absl::flat_hash_set<NodeHandle>& selected_nodes =
            editor.GetSelectedNodes();

        // Go up the tree from each selected node and expand ancestors.
        for (NodeHandle node : selected_nodes) {
          NodeHandle parent = node->GetParent();

          while (parent) {
            manually_expanded_nodes_.insert(parent);
            manually_collapsed_nodes_.erase(parent);
            parent = parent->GetParent();
          }
        }

        selected_nodes_changed_ = true;
      },
      this);

  const auto handle_keyboard_event = [this](const imp::KeyboardEvent& event) {
    // If the delete or backspace key is pressed, delete the selected nodes.
    // Only handle the key press if the user is not typing in a text field.
    if (event.type == KeyboardEventType::kOnDown &&
        (event.key.code == VirtualKeyCode::VK_DELETE ||
         event.key.code == VirtualKeyCode::VK_BACKSPACE) &&
        !ImGui::GetIO().WantTextInput) {
      view_.GetRegistry().GetOrCreate<EditorClipboard>(&view_).Delete();
    }

    // Handle multi-select keys.
    if (absl::c_linear_search(kMultiSelectKeyCodes, event.key.code)) {
      switch (event.type) {
        case KeyboardEventType::kOnDown:
          held_multi_select_keys_.insert(event.key.code);
          break;
        case KeyboardEventType::kOnUp:
          held_multi_select_keys_.erase(event.key.code);
          break;
        default:
          break;
      }
    }

    // Handle shift keys.
    if (absl::c_linear_search(kShiftKeyCodes, event.key.code)) {
      switch (event.type) {
        case KeyboardEventType::kOnDown:
          held_shift_keys_.insert(event.key.code);
          break;
        case KeyboardEventType::kOnUp:
          held_shift_keys_.erase(event.key.code);
          break;
        default:
          break;
      }
    }
  };

  editor.GetDispatcher().Connect(handle_keyboard_event, this);
}

ImGuiTreeNodeFlags Hierarchy::GetTreeNodeFlags() const {
  return ImGuiTreeNodeFlags_DefaultOpen;
}

void Hierarchy::DrawImGui() {
  // Draw the popup menu for creating new nodes.
  if (ImGui::BeginPopupContextWindow()) {
    ShowCreateNodeMenu();
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
  ImGui::PushStyleColor(ImGuiCol_Header, kDarkPrimary);
  ImGui::PushStyleColor(ImGuiCol_HeaderHovered, kDarkLowlight);
  ImGui::PushStyleColor(ImGuiCol_HeaderActive, kDarkLowlight);
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
  ImGui::PopStyleColor();  // ImGuiCol_HeaderActive
  ImGui::PopStyleColor();  // ImGuiCol_HeaderHovered
  ImGui::PopStyleColor();  // ImGuiCol_Header
}

void Hierarchy::DrawHierarchy(
    NodeHandle node, std::optional<RobinSet<NodeHandle>> filtered_nodes,
    const absl::flat_hash_set<NodeHandle>& selected_nodes) {
  if (!ShouldShowNode(node, filtered_nodes)) return;

  if (!DrawNode(node, filtered_nodes, selected_nodes)) return;

  if (node) {
    for (NodeHandle child : node->GetChildren()) {
      // Because of operations like multi-select -> delete, etc, it's possible
      // for a child node to be removed in between DrawHierarchy calls.
      if (!child) continue;

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
  const bool is_node_selected = selected_nodes.contains(node);

  bool should_scroll_to_node = false;
  // Check if the node is currently selected.
  if (is_node_selected) {
    flags |= ImGuiTreeNodeFlags_Selected;
    if (selected_nodes_changed_) {
      ImGui::SetNextItemOpen(true, ImGuiCond_Always);
      should_scroll_to_node = true;
      selected_nodes_changed_ = false;
    }
  }

  // Check if the node has no children, in which case it is a leaf.
  if (node->GetChildren().empty()) {
    flags |= ImGuiTreeNodeFlags_Leaf;
  }

  // If the filter is active, control expanded state by a filter match.
  if (!filtered_nodes.has_value()) {
    // If the filter is empty, set the expanded state to the set of expanded
    // nodes controlled by the user. This restores the hierarchy to its
    // original state if the user deletes the filter.
    manually_collapsed_nodes_.clear();
  }
  ImGui::SetNextItemOpen(IsNodeExpanded(node, filtered_nodes),
                         ImGuiCond_Always);

  const bool is_disabled = !node->IsActive();
  if (is_disabled) {
    ImGui::PushStyleColor(ImGuiCol_Text,
                          ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
  }
  const bool is_expanded =
      ImGui::TreeNodeEx(GetTreeNodeLabelForNode(node).c_str(), flags);
  if (is_disabled) {
    ImGui::PopStyleColor();
  }

  if (should_scroll_to_node) {
    ImGui::SetScrollHereY();
  }

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
  const bool is_mouse_beyond_arrow =
      node->GetChildren().empty() ||
      (ImGui::GetMousePos().x - ImGui::GetItemRectMin().x) >
          ImGui::GetTreeNodeToLabelSpacing();
  // Use IsMouseReleased && IsItemHovered instead of IsItemClicked becaues
  // IsItemClicked is called on mouse down, but we want the selection to occur
  // on mouse-up. This makes it possible to drag and drop nodes from the
  // hierarchy panel to the node_details panel, and is more consistent with
  // the rest of the UX.
  if (ImGui::IsMouseReleased(0) &&
      ImGui::IsItemHovered(ImGuiHoveredFlags_None) && is_mouse_beyond_arrow) {
    if (!held_shift_keys_.empty()) {
      SelectRange(node);
    } else if (!held_multi_select_keys_.empty()) {
      editor.SelectNode(node, EditorInfo::SelectionMode::kMultipleNodes);
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
    ShowCreateNodeMenu();
    ImGui::Separator();

    EditorClipboard& editor_clipboard =
        view_.GetRegistry().GetOrCreate<EditorClipboard>(&view_);
    if (ImGui::MenuItem("Cut")) {
      if (!is_node_selected) {
        editor.SelectNode(node);
      }
      editor_clipboard.Cut();
    }
    if (ImGui::MenuItem("Copy")) {
      if (!is_node_selected) {
        editor.SelectNode(node);
      }
      editor_clipboard.Copy();
    }
    if (!editor_clipboard.IsEmpty() &&
        ImGui::MenuItem(GetPasteLabel(editor_clipboard).c_str())) {
      editor.SelectNode(node);
      editor_clipboard.Paste();
    }
    if (ImGui::MenuItem("Delete")) {
      if (!is_node_selected) {
        editor.SelectNode(node);
      }
      editor_clipboard.Delete();
    }
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
    if (HasValidMesh(node, CameraHelperOptions::kIncludeDescendants) &&
        ImGui::MenuItem("Focus on this node")) {
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
      const auto peeked_metadata = peeked_node->GetComponent<SceneMetadata>();
      const bool is_child_of_base =
          peeked_metadata && peeked_metadata->IsChildOfBase();

      if (is_ancestor_of_target || is_child_of_base) {
        is_valid_target = false;
        break;
      }
    }

    if (is_valid_target) {
      // It's a valid drop target.
      if (ImGui::BeginDragDropTarget()) {
        // Accept the drag & drop payload node, which clears the payload.
        // Ensure that the payload is the same as the one we peeked at
        // earlier.
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

bool Hierarchy::ShouldShowNode(
    NodeHandle node,
    const std::optional<RobinSet<NodeHandle>>& filtered_nodes) const {
  if (filtered_nodes.has_value() && !filtered_nodes->contains(node)) {
    return false;
  }

  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  // Do not draw any of the editor nodes.
  if (node == editor.GetEditorRoot()) {
    return false;
  }

#if IMP_RUNTIME(DEV)
  if (node->IsEditorStaging()) {
    return false;
  }
#endif

  return true;
}

bool Hierarchy::IsNodeExpanded(
    NodeHandle node,
    const std::optional<RobinSet<NodeHandle>>& filtered_nodes) const {
  if (filtered_nodes.has_value()) {
    return filtered_nodes.value().contains(node) &&
           !manually_collapsed_nodes_.contains(node);
  }
  return manually_expanded_nodes_.contains(node);
}

void Hierarchy::SelectRange(NodeHandle end_node) {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  const absl::flat_hash_set<NodeHandle>& selected_nodes =
      editor.GetSelectedNodes();

  // We have to find all the visible nodes in the Hierarchy because range
  // selection needs to work even if a filter is being used.
  std::vector<NodeHandle> visible_nodes;
  view_.ForEachNode(
      [this, &visible_nodes](NodeHandle root) {
        std::optional<RobinSet<NodeHandle>> filtered_nodes = std::nullopt;
        if (filter_.IsActive()) {
          filtered_nodes.emplace();
          CollectFilteredNodes(filter_, *filtered_nodes, root);
        }
        CollectVisibleNodes(root, filtered_nodes, visible_nodes);
      },
      NodeFlags::kIsRoot);

  // Find the selected node that is furthest away from the end node.
  // This will make it so that if you already have multiple nodes selected,
  // when you shift-click a new node, it will additively select the range.
  auto start_it = visible_nodes.end();
  auto end_it = absl::c_find(visible_nodes, end_node);

  int max_distance = -1;
  int end_index = std::distance(visible_nodes.begin(), end_it);

  for (int i = 0; i < visible_nodes.size(); ++i) {
    if (!selected_nodes.contains(visible_nodes[i])) continue;

    int distance = std::abs(i - end_index);

    if (distance <= max_distance) continue;

    max_distance = distance;
    start_it = visible_nodes.begin() + i;
  }

  // If no visible start node was found, select the end node only.
  // This happens if the start node was filtered out since it was selected.
  if (start_it == visible_nodes.end()) {
    editor.SelectNode(end_node);
    return;
  }

  // If the start node is after the end node, swap them.
  // Required for ranges from a low node to a high node rather than vice-versa.
  if (start_it > end_it) {
    std::swap(start_it, end_it);
  }

  // Select the whole range of nodes we've found.
  editor.SelectNode(*start_it);
  for (auto it = start_it + 1; it <= end_it; ++it) {
    editor.SelectNode(*it, EditorInfo::SelectionMode::kMultipleNodes);
  }
}

void Hierarchy::CollectVisibleNodes(
    NodeHandle node, const std::optional<RobinSet<NodeHandle>>& filtered_nodes,
    std::vector<NodeHandle>& out_nodes) {
  if (!ShouldShowNode(node, filtered_nodes)) return;

  out_nodes.push_back(node);

  if (!IsNodeExpanded(node, filtered_nodes)) return;

  // Recursively collect all expanded, non-filtered children.
  for (NodeHandle child : node->GetChildren()) {
    if (!child) continue;

    CollectVisibleNodes(child, filtered_nodes, out_nodes);
  }
}

void Hierarchy::ShowCreateNodeMenu() {
  if (ImGui::BeginMenu("New Node")) {
    if (ImGui::MenuItem("Empty")) {
      CreateAndSelectEmptyNode(view_);
    }
    if (ImGui::MenuItem("Cube")) {
      CreateAndSelectPrimitiveShapeNode(view_, PrimitiveShapeType::kBox);
    }
    if (ImGui::MenuItem("Sphere")) {
      CreateAndSelectPrimitiveShapeNode(view_, PrimitiveShapeType::kSphere);
    }
    if (ImGui::MenuItem("Cone")) {
      CreateAndSelectPrimitiveShapeNode(view_, PrimitiveShapeType::kCone);
    }
    if (ImGui::MenuItem("Cylinder")) {
      CreateAndSelectPrimitiveShapeNode(view_, PrimitiveShapeType::kCylinder);
    }
    if (ImGui::MenuItem("Capsule")) {
      CreateAndSelectPrimitiveShapeNode(view_, PrimitiveShapeType::kCapsule);
    }
    if (ImGui::MenuItem("Quad")) {
      CreateAndSelectPrimitiveShapeNode(view_, PrimitiveShapeType::kQuad);
    }
    ImGui::EndMenu();
  }
}

}  // namespace imp::editor
