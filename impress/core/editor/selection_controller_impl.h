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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_SELECTION_CONTROLLER_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_SELECTION_CONTROLLER_IMPL_H_

#include <vector>

#include "core/editor/selection_controller.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/system.h"

namespace imp::editor {

// Contains node selection logic within Editor
class SelectionControllerImpl : public SelectionController, public System {
 public:
  // Once constructed, SelectionController listens for TapEvents and sends
  // NodeSelectionChangedEvents, cycling through all intersected nodes on
  // subsequent TapEvents. Events are listened and sent through the Editor
  // dispatcher. See implementation for a more detailed explanation.
  explicit SelectionControllerImpl(BaseView* view);

  // Returns the currently selected ndde.
  // The returned NodeHandle will be invalid if nothing is selected.
  NodeHandle GetSelectedNode() const override;

  // Returns true if the given node is selected or is an ancestor of the
  // selected node.
  bool IsNodeOrAncestorSelected(NodeHandle node) override;

  // Notifies that a node has been selected by sending a
  // NodeSelectionChangedEvent with the provided node as the target. If the node
  // is already selected, no event will be sent.
  void TrySelectNode(NodeHandle node_to_select) override;

 private:
  void CycleOrUpdateSelection(NodeHandle target,
                              std::vector<NodeHandle> intersecting_nodes);

  // Moves the camera to focus and frame it on the current selected node.
  void FocusCameraOnSelection(NodeHandle target);

  NodeHandle selected_node_;
  std::vector<NodeHandle> selectable_nodes_;
  int selected_node_index_ = 0;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_SELECTION_CONTROLLER_IMPL_H_
