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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_CLIPBOARD_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_CLIPBOARD_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/editor/events.h"
#include "core/input/keyboard_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_data.proto.imp.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/system.h"

namespace imp::editor {

// The EditorClipboard handles clipboard-related editor actions such as copying
// and pasting nodes.
class EditorClipboard : public System {
 public:
  EditorClipboard(BaseView* view) : System(view) {}

  // Begins listening for clipboard-related actions through the given
  // dispatcher.
  void EnableDispatcherEvents(Dispatcher& dispatcher);

  // Handles a clipboard "cut" action, which only supports NodeHandles.
  // Supports "undo", but not "redo".
  // TODO: Consider supporting "redo".
  void Cut();

  // Handles a clipboard "copy" action, which only supports NodeHandles.
  // Supports neither "undo" nor "redo".
  // TODO: Consider supporting "redo".
  void Copy();

  // Handles a clipboard "paste" action, which only supports NodeHandles.
  // Supports "undo", but not "redo".
  // If the node selection *does not* change between the cut/copy and paste
  // actions, the node will be pasted as a sibling of the copied node.
  // If the node selection *does* change between the cut/copy and paste actions,
  // the node will be pasted as a child of the newly-selected node.
  // The name of the pasted node will be the original name of the copied node,
  // with a " (n)" suffix, where n is the smallest integer such that the name
  // is unique within the new parent node's children.
  // TODO: Consider supporting "redo".
  void Paste();

  // Deletes all selected nodes. Supports "undo", but not "redo".
  // Delete is essentially the inverse of paste: undoing a paste is equivalent
  // to a delete, and undoing a delete is equivalent to a paste.
  void Delete();

  // Returns true if the clipboard is empty.
  bool IsEmpty() const;

  // Returns the names of the nodes on the clipboard.
  std::vector<absl::string_view> GetClipboardNodeNames() const;

 private:
  // A struct holding the data needed to recreate a node from the clipboard.
  struct ClipboardNode {
    // Creates a ClipboardNode from a NodeHandle. May fail if the node is
    // invalid, or is missing the required metadata.
    static absl::StatusOr<ClipboardNode> FromNodeHandle(const NodeHandle& node);
    // Creates a std::vector<ClipboardNode> from a std::vector<NodeHandle>. May
    // fail if any of the nodes are invalid or missing the required metadata.
    static absl::StatusOr<std::vector<ClipboardNode>> FromVector(
        const std::vector<NodeHandle>& nodes);

    // The original parent node of the copied node.
    NodeHandle parent_node_handle;
    // The original NodeHandle of the copied node.
    NodeHandle node_handle;
    // The serialized data of the node.
    NodeData node_data;
    // Needed to preserve metadata about the original node.
    std::string asset_url;
  };

  // Saves the given clipboard nodes to the clipboard. Returns an error if any
  // of the nodes are invalid.
  absl::Status CopyImpl(const std::vector<ClipboardNode>& clipboard_nodes);

  // Asynchronously pastes the given clipboard nodes. Returns a future that will
  // be resolved when all the nodes are pasted. If a node_parent is provided,
  // the node will be parented to that node. Otherwise, the node will be
  // parented to the ClipboardNode's original parent.
  Future<std::vector<NodeHandle>> PasteImpl(
      const std::vector<ClipboardNode>& clipboard_nodes,
      const std::optional<NodeHandle>& node_parent = std::nullopt);

  // Deletes the node from the scene for each given ClipboardNode.
  // Note: this only deletes the corresponding NodeHandle from the scene. A copy
  // of the NodeData remains in the ClipboardNode.
  absl::Status DeleteImpl(const std::vector<ClipboardNode>& nodes_to_delete);

  // Deselects all selected nodes and destroys the given NodeHandles.
  absl::Status DeleteImpl(const std::vector<NodeHandle>& nodes_to_delete);

  // Deselects all selected nodes, ignoring the following
  // NodeSelectionChangedEvent.
  void Deselect();

  // Discerns clipboard hotkeys and triggers the appropriate clipboard action.
  void HandleKeyboardEvent(const KeyboardEvent& event);

  // Listens for node selection changes and updates the paste node parent.
  void HandleNodeSelectionChangedEvent(const NodeSelectionChangedEvent& event);

  // A list of ClipboardNodes to copy.
  std::vector<ClipboardNode> copied_nodes_;
  // If set, all pasted nodes will be parented to this node.
  std::optional<NodeHandle> paste_node_parent_;
  // Whether to ignore the next NodeSelectionChangedEvent. Used to select the
  // pasted node without setting it as the parent of the next pasted node.
  bool ignore_next_selection_changed_event_ = false;
  // Whether or not to ignore dispatcher events.
  bool ignore_keyboard_input_ = false;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_CLIPBOARD_H_
