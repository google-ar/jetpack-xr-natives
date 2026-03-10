/*
 * Copyright 2025 Google LLC
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

#include "core/editor/editor_clipboard.h"

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/common/robin_set.h"
#include "core/editor/command_manager.h"
#include "core/editor/editor_info.h"
#include "core/editor/events.h"
#include "core/editor/function_command.h"
#include "core/editor/selection_controller.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_data.proto.imp.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/scene/scene_reference.h"
#include "core/view/framework/scene/scene_system.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::editor {

namespace {

// Returns true if any of the nodes are invalid.
bool IsAnyInvalid(const std::vector<NodeHandle>& nodes) {
  for (const NodeHandle& node : nodes) {
    if (!node) {
      return true;
    }
  }
  return false;
}

// Returns true if node is a descendant of any of the nodes in
// potential_ancestors.
bool IsDescendantOfAny(
    const NodeHandle& node,
    const absl::flat_hash_set<NodeHandle>& potential_ancestors) {
  NodeHandle parent = node->GetParent();
  while (parent) {
    if (potential_ancestors.contains(parent)) {
      return true;
    }
    parent = parent->GetParent();
  }
  return false;
}

// Returns only the top-most NodeHandles in the selection. In other words,
// if any of the selected nodes are descendants of other selected nodes,
// those descendants will not be returned.
// This is useful because copying a parent node will copy the entire subtree
// anyways, so copying the descendants would cause unintended duplicates.
std::vector<NodeHandle> GetTopLevelSelectedNodes(BaseView& view) {
  const absl::flat_hash_set<NodeHandle> selected_nodes =
      view.GetRegistry().Get<SelectionController>()->get().GetSelectedNodes();
  std::vector<NodeHandle> nodes_to_copy;
  for (const NodeHandle& node : selected_nodes) {
    if (IsDescendantOfAny(node, selected_nodes)) {
      continue;
    }
    nodes_to_copy.push_back(node);
  }
  return nodes_to_copy;
}

// Returns a unique name for the node to be pasted as a child of node_parent.
// If the name of the node_handle is already unique among node_parent's
// children, it is returned.
// Otherwise, we return "<original_name> (n)", where n is the smallest integer
// such that the name is unique among the names of node_parent's siblings.
std::string GenerateUniqueSiblingName(const NodeHandle& node_handle,
                                      const NodeHandle& node_parent) {
  const std::string& original_name = std::string(node_handle->GetName());
  if (node_parent && node_parent->GetChildren().empty()) {
    // If there are no siblings, we can just use the original name.
    return original_name;
  }

  // Collect the names of all to-be siblings.
  RobinSet<absl::string_view> sibling_names;
  if (node_parent) {
    for (const NodeHandle& sibling : node_parent->GetChildren()) {
      sibling_names.insert(sibling->GetName());
    }
  } else {
    for (const NodeHandle& root_node :
         node_handle->GetView().GetPathManager().GetRootNodes()) {
      sibling_names.insert(root_node->GetName());
    }
  }

  // If the original name is unique, we can just use it.
  if (!sibling_names.contains(original_name)) {
    return original_name;
  }

  // Otherwise, find the smallest integer n such that the name "original_name
  // (n)" is unique.
  const int num_unique_names = sibling_names.size();
  for (int i = 0; i < num_unique_names; ++i) {
    const std::string candidate_name =
        absl::StrCat(original_name, " (", (i + 1), ")");
    if (!sibling_names.contains(candidate_name)) {
      return candidate_name;
    }
  }

  // This is impossible, because we know one of the nodes has original_name,
  // which means there are exactly (num_unique_names - 1) names that are not
  // original_name, but the loop above runs exactly num_unique_names times.
  // In other words, if the loop completes without returning, it implies there
  // are (num_unique_names + 1) unique names, which is False.
  return original_name;
}
}  // namespace

absl::StatusOr<EditorClipboard::ClipboardNode>
EditorClipboard::ClipboardNode::FromNodeHandle(const NodeHandle& node) {
  if (!node) {
    return absl::InvalidArgumentError("Node is invalid.");
  }

  MP_ASSIGN_OR_RETURN(const NodeData node_data,
                   node->GetView().GetSceneSystem().SaveToData(
                       node, SceneSystem::SaveMode::kAuthoredContent));

  // Required to preserve metadata about the original node.
  auto scene_reference = node->GetComponent<SceneReference>();
  const std::string asset_url =
      scene_reference ? std::string(scene_reference->GetAssetUrl()) : "";

  return ClipboardNode{.parent_node_handle = node->GetParent(),
                       .node_handle = node,
                       .node_data = node_data,
                       .asset_url = asset_url};
}

absl::StatusOr<std::vector<EditorClipboard::ClipboardNode>>
EditorClipboard::ClipboardNode::FromVector(
    const std::vector<NodeHandle>& nodes_to_copy) {
  std::vector<ClipboardNode> copied_nodes;
  copied_nodes.reserve(nodes_to_copy.size());
  for (const NodeHandle& node_to_copy : nodes_to_copy) {
    MP_ASSIGN_OR_RETURN(ClipboardNode clipboard_node,
                     ClipboardNode::FromNodeHandle(node_to_copy));
    copied_nodes.push_back(std::move(clipboard_node));
  }
  return copied_nodes;
}

void EditorClipboard::EnableDispatcherEvents(Dispatcher& dispatcher) {
  // Ignore keyboard input when the editor is disabled.
  dispatcher.Connect(
      [this](const EditorEnabledEvent& event) {
        ignore_keyboard_input_ = !event.enabled;
      },
      this);
  // Register keyboard event handlers.
  dispatcher.Connect(
      [this](const KeyboardEvent& event) {
        if (ignore_keyboard_input_) {
          return;
        }
        HandleKeyboardEvent(event);
      },
      this);
  // Listen for node selection changes.
  dispatcher.Connect(
      [this](const NodeSelectionChangedEvent& event) {
        HandleNodeSelectionChangedEvent(event);
      },
      this);
}

void EditorClipboard::HandleKeyboardEvent(const imp::KeyboardEvent& event) {
  // CTRL or GUI must be held, and the event must be a key down.
  if (event.type != KeyboardEventType::kOnDown ||
      !HasKeyModifier(KeyModifier::CTRL_OR_GUI, event.key.modifiers)) {
    return;
  }
  switch (event.key.code) {
    // Standard hotkey for "cut".
    case VirtualKeyCode::VK_x: {
      Cut();
      return;
    }
    // Standard hotkey for "copy".
    case VirtualKeyCode::VK_c: {
      Copy();
      return;
    }
    // Standard hotkey for "paste".
    case VirtualKeyCode::VK_v: {
      Paste();
      return;
    }
    default:
      return;
  }
}

void EditorClipboard::HandleNodeSelectionChangedEvent(
    const NodeSelectionChangedEvent& event) {
  // When a node is selected, we consider it the parent of the node to paste,
  // unless flagged otherwise, such as when we select a just-pasted node.
  if (ignore_next_selection_changed_event_) {
    ignore_next_selection_changed_event_ = false;
    return;
  }
  const absl::flat_hash_set<NodeHandle> selected_nodes =
      GetView()
          .GetRegistry()
          .Get<SelectionController>()
          ->get()
          .GetSelectedNodes();
  if (selected_nodes.empty()) {
    // Paste the node at the root of the scene if no node is selected.
    paste_node_parent_ = NodeHandle();
    return;
  }
  // Otherwise, paste the node as a child of the first selected node.
  paste_node_parent_ = *selected_nodes.begin();
}

void EditorClipboard::Cut() {
  // No-op if no node is selected.
  std::vector<NodeHandle> nodes_to_cut = GetTopLevelSelectedNodes(GetView());
  if (nodes_to_cut.empty()) {
    return;
  }

  absl::StatusOr<std::vector<ClipboardNode>> clipboard_nodes =
      ClipboardNode::FromVector(nodes_to_cut);
  if (!clipboard_nodes.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to cut nodes: " << clipboard_nodes.status();
    return;
  }

  // A functor to perform the cut command, which will be executed by the
  // CommandManager, and paired with the undo_cut functor.
  const auto perform_cut = [this, clipboard_nodes]() -> absl::Status {
    // A cut is a copy followed by a delete.
    MP_RETURN_IF_ERROR(CopyImpl(*clipboard_nodes));
    MP_RETURN_IF_ERROR(DeleteImpl(*clipboard_nodes));
    return absl::OkStatus();
  };

  // Restores the nodes that were cut.
  const auto undo_cut = [this, clipboard_nodes]() {
    // Undoing a cut is just a paste.
    PasteImpl(*clipboard_nodes).KeptBy(this);
  };

  // Execute the cut command.
  GetView()
      .GetRegistry()
      .GetOrCreate<CommandManager>()
      .PerformCommand<FunctionCommand>(perform_cut, undo_cut);
}

void EditorClipboard::Copy() {
  // No-op if nothing is selected.
  std::vector<NodeHandle> nodes_to_copy = GetTopLevelSelectedNodes(GetView());
  if (nodes_to_copy.empty()) {
    return;
  }
  absl::StatusOr<std::vector<ClipboardNode>> clipboard_nodes =
      ClipboardNode::FromVector(nodes_to_copy);

  if (!CopyImpl(*clipboard_nodes).ok()) {
    IMP_LOG(imp::ERROR) << "Failed to copy nodes: "
               << clipboard_nodes.status().ToString();
  }
}

void EditorClipboard::Paste() {
  if (copied_nodes_.empty()) {
    IMP_LOG(imp::WARNING) << "No nodes to paste.";
    return;
  }

  // A future that will be resolved when all the nodes are pasted.
  Future<std::vector<NodeHandle>> paste_future;
  // A functor to perform the paste command, which will be executed by the
  // CommandManager, and paired with the undo_paste functor.
  const auto perform_paste = [this, copied_nodes = copied_nodes_, paste_future,
                              paste_node_parent =
                                  paste_node_parent_]() mutable {
    // Deselect previous nodes before pasting.
    Deselect();
    if (copied_nodes.empty()) {
      return absl::InternalError("No nodes to paste.");
    }
    if (paste_future.Ready()) {
      return absl::InternalError("Paste does not support redo.");
    }
    PasteImpl(copied_nodes, paste_node_parent)
        .Then([paste_future](
                  absl::StatusOr<std::vector<NodeHandle>> pasted_nodes) {
          paste_future.Return(pasted_nodes);
        })
        .KeptBy(this);
    return absl::OkStatus();
  };

  // Deletes the nodes that were pasted.
  const auto undo_paste = [this, paste_future]() {
    // "If the paste future is ready and any of the nodes are invalid..."
    if (paste_future.Ready() && paste_future.Get().ok() &&
        IsAnyInvalid(paste_future.Get().value())) {
      return absl::InternalError(
          "Cannot undo paste: NodeHandle is now invalid.");
    }
    paste_future
        .Then([this](std::vector<NodeHandle> pasted_nodes) {
          return DeleteImpl(pasted_nodes);
        })
        .KeptBy(this);
    return absl::OkStatus();
  };

  // Execute the paste.
  GetView()
      .GetRegistry()
      .GetOrCreate<CommandManager>()
      .PerformCommand<FunctionCommand>(perform_paste, undo_paste);
}

void EditorClipboard::Delete() {
  std::vector<NodeHandle> nodes_to_delete = GetTopLevelSelectedNodes(GetView());
  if (nodes_to_delete.empty()) {
    return;
  }
  std::vector<ClipboardNode> clipboard_nodes =
      ClipboardNode::FromVector(nodes_to_delete)
          .value_or(std::vector<ClipboardNode>());

  // A functor to perform the delete command, which will be executed by the
  // CommandManager, and paired with the undo_delete functor.
  const auto perform_delete = [this, nodes_to_delete]() -> absl::Status {
    return DeleteImpl(nodes_to_delete);
  };
  // Undoing a delete is identical to a paste.
  const auto undo_delete = [this, clipboard_nodes]() {
    PasteImpl(clipboard_nodes).KeptBy(this);
    return absl::OkStatus();
  };

  GetView()
      .GetRegistry()
      .GetOrCreate<CommandManager>()
      .PerformCommand<FunctionCommand>(perform_delete, undo_delete);
}

Future<std::vector<NodeHandle>> EditorClipboard::PasteImpl(
    const std::vector<ClipboardNode>& clipboard_nodes,
    const std::optional<NodeHandle>& node_parent) {
  // Deselect previous nodes before pasting.
  Deselect();
  // Collect the futures for each pasted node.
  std::vector<Future<NodeHandle>> paste_futures;
  for (const ClipboardNode& clipboard_node : clipboard_nodes) {
    Future<NodeHandle> paste_future = Future<NodeHandle>();
    NodeHandle parent = node_parent.value_or(clipboard_node.parent_node_handle);
    paste_futures.push_back(
        GetView()
            .GetSceneSystem()
            .LoadScene(
                clipboard_node.node_data, clipboard_node.asset_url,
                SceneSystem::LoadSceneOptions{
                    .metadata_mode = SceneSystem::MetadataMode::kInclude})
            .Then([this, parent](NodeHandle node_handle) {
              node_handle->SetName(
                  GenerateUniqueSiblingName(node_handle, parent));
              node_handle->SetParent(parent);
              // When we paste, we want to select the new node, but we don't
              // want to set it as the parent of the next pasted node.
              ignore_next_selection_changed_event_ = true;
              node_handle->GetView()
                  .GetRegistry()
                  .Get<SelectionController>()
                  ->get()
                  .TrySelectNode(node_handle,
                                 EditorInfo::SelectionMode::kMultipleNodes);
              return node_handle;
            }));
  }
  return Future<NodeHandle>::MergeList(paste_futures);
}

absl::Status EditorClipboard::CopyImpl(
    const std::vector<ClipboardNode>& clipboard_nodes) {
  for (const ClipboardNode& clipboard_node : clipboard_nodes) {
    if (!clipboard_node.node_handle) {
      return absl::InternalError("Node is invalid.");
    }
  }
  // Indicates that we should paste to the parents of the copied nodes.
  paste_node_parent_ = std::nullopt;
  // Copy the nodes to the clipboard.
  copied_nodes_ = clipboard_nodes;
  return absl::OkStatus();
}

absl::Status EditorClipboard::DeleteImpl(
    const std::vector<ClipboardNode>& nodes_to_delete) {
  std::vector<NodeHandle> node_handles;
  node_handles.reserve(nodes_to_delete.size());
  for (const ClipboardNode& clipboard_node : nodes_to_delete) {
    node_handles.push_back(clipboard_node.node_handle);
  }
  return DeleteImpl(node_handles);
}

absl::Status EditorClipboard::DeleteImpl(
    const std::vector<NodeHandle>& nodes_to_delete) {
  if (IsAnyInvalid(nodes_to_delete)) {
    return absl::InternalError("Cannot delete invalid node.");
  }
  for (const NodeHandle& node_handle : nodes_to_delete) {
    GetView().DestroyNode(node_handle);
  }
  return absl::OkStatus();
}

void EditorClipboard::Deselect() {
  ignore_next_selection_changed_event_ = true;
  GetView().GetRegistry().Get<SelectionController>()->get().TrySelectNode(
      NodeHandle());
}

bool EditorClipboard::IsEmpty() const { return copied_nodes_.empty(); }

std::vector<absl::string_view> EditorClipboard::GetClipboardNodeNames() const {
  std::vector<absl::string_view> node_names;
  node_names.reserve(copied_nodes_.size());
  for (const ClipboardNode& copied_node : copied_nodes_) {
    node_names.push_back(copied_node.node_data.name);
  }
  return node_names;
}

}  // namespace imp::editor
