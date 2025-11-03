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
// TODO: Support multiple node selection
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

  // Returns whether there is a node on the clipboard.
  bool HasClipboardNode() const;

  // Returns the name of the node on the clipboard, if there is a node on the
  // clipboard.
  std::optional<absl::string_view> GetClipboardNodeName() const;

 private:
  // A struct holding the data needed to recreate a node from the clipboard.
  struct ClipboardNode {
    // Creates a ClipboardNode from a NodeHandle. May fail if the node is
    // invalid, or is missing the required metadata.
    static absl::StatusOr<ClipboardNode> FromNodeHandle(const NodeHandle& node);
    NodeData node_data;
    // Needed to preserve metadata about the original node.
    std::string asset_url;
  };

  // Copies a NodeHandle to the clipboard (overlapping functionality
  // of both "cut" and "copy").
  void CopyImpl(const NodeHandle& node_to_copy);

  // Adds a NodeHandle to the scene, based on the clipboard_node, and parented
  // under the given node_parent.
  Future<NodeHandle> CreateNodeHandleFromClipboardNode(
      const ClipboardNode& clipboard_node, const NodeHandle& node_parent);

  // Discerns clipboard hotkeys and triggers the appropriate clipboard action.
  void HandleKeyboardEvent(const KeyboardEvent& event);

  // Listens for node selection changes and updates the paste node parent.
  void HandleNodeSelectionChangedEvent(const NodeSelectionChangedEvent& event);

  // The ClipboardNode currently on the clipboard, if any.
  std::optional<ClipboardNode> copied_node_;
  // The parent node to use when pasting a node from the clipboard.
  NodeHandle paste_node_parent_;
  // Whether to ignore the next NodeSelectionChangedEvent. Used to select the
  // pasted node without setting it as the parent of the next pasted node.
  bool ignore_next_selection_changed_event_ = false;
  // Whether or not a paste is currently pending.
  bool pending_paste_ = false;
  // Whether or not to ignore dispatcher events.
  bool ignore_keyboard_input_ = false;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_CLIPBOARD_H_
