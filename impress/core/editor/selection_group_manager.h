// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_SELECTION_GROUP_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_SELECTION_GROUP_MANAGER_H_

#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/strings/string_view.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

// The group nodes are added to when selected, used by the outline renderer.
inline constexpr absl::string_view kSelectedGroup = "EditorSelected";
inline constexpr absl::string_view kSelectionGroupManagerTypeUrl =
    "imp.editor.SelectionGroupManager";

// Tracks Nodes selected in the editor and adds them to a specific group.
// This group is used for a TexturePipelineRenderer to render outlines.
class SelectionGroupManager : public Component {
 public:
  using IsfInfo =
      StatelessIsfInfo<SelectionGroupManager, kSelectionGroupManagerTypeUrl>;

  void Setup() {
    GetView().GetRegistry().Get<Editor>()->get().GetDispatcher().Connect(
        [this](const NodeSelectionChangedEvent& event) {
          Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
          const absl::flat_hash_set<NodeHandle>& selected =
              editor.GetSelectedNodes();

          for (const NodeHandle& node : selected_nodes_) {
            node->RemoveFromGroup(kSelectedGroup);
          }

          selected_nodes_.assign(selected.begin(), selected.end());

          for (const NodeHandle& node : selected_nodes_) {
            node->AddToGroup(kSelectedGroup);
          }
        },
        this);
  }

 private:
  std::vector<NodeHandle> selected_nodes_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_SELECTION_GROUP_MANAGER_H_
