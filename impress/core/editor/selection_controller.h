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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_SELECTION_CONTROLLER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_SELECTION_CONTROLLER_H_

#include "absl/container/flat_hash_set.h"
#include "core/editor/editor.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

// Selection controller APIs. This is an interface that is used to select and
// deselect nodes in the scene in the editor. It is virtual, so that it can be
// used within framework code, avoiding a circular dependency between framework
// and editor code.
//
// Example usage:
//
//   imp::NodeHandle selected_node =
//       GetRegistry().Get<editor::SelectionController>()->GetSelectedNode();
//   if (selected_node.IsValid()) {
//     // e.g. visualize the renderable of the selected node.
//   }
struct SelectionController {
 public:
  virtual ~SelectionController() = default;

  // Returns the currently selected nodes.
  virtual const absl::flat_hash_set<NodeHandle>& GetSelectedNodes() = 0;

  // Notifies that a node has been selected by sending a
  // NodeSelectionChangedEvent with the provided node as the target.
  // * If `multi_selection_enabled` is true:
  //     - The node is added to the current selection.
  //     - If the node is already selected, it is deselected.
  // * Otherwise:
  //     - The node becomes the *only* selected node.
  // When the node is invalid, deselect all nodes.
  virtual void TrySelectNode(NodeHandle node_to_select,
                             Editor::SelectionMode selection_mode =
                                 Editor::SelectionMode::kSingleNode) = 0;

  // Disables selecting the model on ModelLoadedEvents. Enabled by default.
  virtual void DisableSelectModelWhenLoaded(
      bool disable_select_model_on_load) = 0;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_SELECTION_CONTROLLER_H_
