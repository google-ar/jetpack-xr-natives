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

// Returns the first selected node, or an error if no nodes are selected.
absl::StatusOr<NodeHandle> GetFirstSelectedNode(BaseView& view) {
  const absl::flat_hash_set<NodeHandle>& selected_nodes =
      view.GetRegistry().Get<SelectionController>()->get().GetSelectedNodes();
  if (selected_nodes.empty()) {
    return absl::NotFoundError("No nodes are selected.");
  }
  return *selected_nodes.begin();
}

// Generates a unique name for node_handle, which is to be parented to
// node_parent, based on the existing children of node_parent.
// Returns the name of the node_handle, with a " (n)" suffix, where n is the
// smallest integer such that the name is unique.
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

  return ClipboardNode{.node_data = node_data, .asset_url = asset_url};
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
  // Paste the node at the root of the scene if no node is selected.
  paste_node_parent_ = GetFirstSelectedNode(GetView()).value_or(NodeHandle());
}

void EditorClipboard::Cut() {
  // No-op if no node is selected.
  absl::StatusOr<NodeHandle> node_to_cut_status =
      GetFirstSelectedNode(GetView());
  if (!node_to_cut_status.ok()) {
    return;
  }
  NodeHandle node_to_cut = *node_to_cut_status;

  // A functor to perform the cut command, which will be executed by the
  // CommandManager, and paired with the undo_cut functor.
  const auto perform_cut = [this, node_to_cut]() -> absl::Status {
    if (!node_to_cut) {
      // This will happen if we are attempting to "redo" a cut, which is
      // unsupported.
      return absl::InternalError("Selected node is invalid.");
    }
    // A cut is a copy followed by a destroy.
    CopyImpl(node_to_cut);
    GetView().DestroyNode(node_to_cut);
    return absl::OkStatus();
  };

  // Restores the node that was cut and the previous state of the clipboard.
  const auto undo_cut = [this, last_copied_node = copied_node_,
                         paste_node_parent = node_to_cut->GetParent()]() {
    if (!copied_node_) {
      return absl::InternalError("No node to undo.");
    }
    CreateNodeHandleFromClipboardNode(*copied_node_, paste_node_parent)
        .Then([this, last_copied_node](NodeHandle node_handle) {
          // This allows chains of cuts to be undone.
          // (i.e., "cut", "cut", "undo", "undo")
          copied_node_ = last_copied_node;
        })
        .KeptBy(this);
    return absl::OkStatus();
  };

  // Execute the cut command.
  GetView()
      .GetRegistry()
      .GetOrCreate<CommandManager>()
      .PerformCommand<FunctionCommand>(perform_cut, undo_cut);
}

void EditorClipboard::Copy() {
  // No-op if no node is selected.
  absl::StatusOr<NodeHandle> node_to_copy = GetFirstSelectedNode(GetView());
  if (!node_to_copy.ok()) {
    return;
  }
  CopyImpl(*node_to_copy);
}

void EditorClipboard::CopyImpl(const NodeHandle& node_to_copy) {
  // We should paste under the same parent by default.
  paste_node_parent_ = node_to_copy->GetParent();

  // Save the node to the clipboard, but log any error here, because this
  // should succeed if the node has the required metadata.
  const absl::StatusOr<ClipboardNode> clipboard_node =
      ClipboardNode::FromNodeHandle(node_to_copy);
  if (!clipboard_node.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to copy node to clipboard: "
               << clipboard_node.status();
    copied_node_ = std::nullopt;
    return;
  }
  copied_node_ = *clipboard_node;
}

void EditorClipboard::Paste() {
  if (!copied_node_) {
    IMP_LOG(imp::WARNING) << "No node to paste.";
    return;
  }
  // We only support one paste at a time to avoid race conditions.
  if (pending_paste_) {
    IMP_LOG(imp::WARNING) << "A paste is already pending.";
    return;
  }
  pending_paste_ = true;

  // A functor to perform the paste command, which will be executed by the
  // CommandManager, and paired with the undo_paste functor.
  Future<NodeHandle> paste_future;
  const auto perform_paste = [this, paste_future]() {
    if (!copied_node_) {
      return absl::InternalError("No node to paste.");
    }
    if (paste_future.Ready()) {
      return absl::InternalError("Paste does not support redo.");
    }
    CreateNodeHandleFromClipboardNode(*copied_node_, paste_node_parent_)
        .Then([this,
               paste_future](absl::StatusOr<NodeHandle> pasted_node_status) {
          // Notify the "undo" lambda of the pasted node.
          paste_future.Return(pasted_node_status);
          pending_paste_ = false;
        })
        .KeptBy(this);
    return absl::OkStatus();
  };

  // Deletes the node that was pasted.
  const auto undo_paste = [this, paste_future]() {
    // "If the paste future is ready and the node is invalid..."
    if (paste_future.Ready() && paste_future.Get().ok() &&
        !paste_future.Get().value()) {
      return absl::InternalError("Can only undo a paste once.");
    }
    paste_future
        .Then([](NodeHandle node_handle) {
          node_handle->GetView().DestroyNode(node_handle);
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

bool EditorClipboard::HasClipboardNode() const {
  return copied_node_.has_value();
}

std::optional<absl::string_view> EditorClipboard::GetClipboardNodeName() const {
  if (!copied_node_.has_value()) {
    return std::nullopt;
  }
  return copied_node_->node_data.name;
}

Future<NodeHandle> EditorClipboard::CreateNodeHandleFromClipboardNode(
    const ClipboardNode& clipboard_node, const NodeHandle& node_parent) {
  return GetView()
      .GetSceneSystem()
      .LoadScene(clipboard_node.node_data, clipboard_node.asset_url,
                 SceneSystem::LoadSceneOptions{
                     .metadata_mode = SceneSystem::MetadataMode::kInclude})
      .Then([this, node_parent](NodeHandle node_handle) {
        node_handle->SetName(
            GenerateUniqueSiblingName(node_handle, node_parent));
        node_handle->SetParent(node_parent);
        // When we paste, we want to select the new node, but we don't want to
        // set it as the parent of the next pasted node.
        ignore_next_selection_changed_event_ = true;
        node_handle->GetView()
            .GetRegistry()
            .Get<SelectionController>()
            ->get()
            .TrySelectNode(node_handle);
        return node_handle;
      });
}
}  // namespace imp::editor
