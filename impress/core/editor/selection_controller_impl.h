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

#include "absl/container/flat_hash_set.h"
#include "core/editor/editor.h"
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

  // Returns the currently selected nodes.
  const absl::flat_hash_set<NodeHandle>& GetSelectedNodes() override;

  // Notifies that a node has been selected by sending a
  // NodeSelectionChangedEvent with the provided node as the target.
  // * If `multi_selection_enabled` is true:
  //     - The node is added to the current selection.
  //     - If the node is already selected, it is deselected.
  // * Otherwise:
  //     - The node becomes the *only* selected node.
  // When the node is invalid, deselect all nodes.
  void TrySelectNode(NodeHandle node_to_select,
                     Editor::SelectionMode selection_mode) override;

  // Disables selecting the model on ModelLoadedEvents. Enabled by default.
  void DisableSelectModelWhenLoaded(bool disable_select_model_on_load) {
    disable_select_model_on_load_ = disable_select_model_on_load;
  }

 private:
  void CycleOrUpdateSelection(NodeHandle target,
                              std::vector<NodeHandle> intersecting_nodes);

  // Moves the camera to focus and frame it on the current selected nodes.
  void FocusCameraOnSelection();

  absl::flat_hash_set<NodeHandle> selected_nodes_;
  std::vector<NodeHandle> selectable_nodes_;
  int selected_node_index_ = 0;
  bool disable_select_model_on_load_ = false;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_SELECTION_CONTROLLER_IMPL_H_
