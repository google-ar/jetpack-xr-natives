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

#include "core/editor/selection_controller_impl.h"

#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/camera/camera_helpers.h"
#include "core/view/framework/gestures/tap_gesture.h"

namespace imp::editor {

SelectionControllerImpl::SelectionControllerImpl(BaseView* view)
    : System(view) {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  PathManager& path_manager = GetView().GetPathManager();

  // Handle TapEvents.
  Dispatcher& editor_dispatcher = editor.GetDispatcher();
  editor_dispatcher.Connect(
      [this, &editor,
       &path_manager](const imp::TapGesture::TapEvent& event) mutable {
        NodeHandle editor_root_node = editor.GetEditorRoot();
        // Create NodeSelectionChangedEvents.
        NodeHandle target;
        // Don't select nodes that are part of the editor.
        if (path_manager.IsAncestorOf(editor_root_node,
                                      event.GetOriginatingNode())) {
          target = event.GetOriginatingNode();
        }

        CycleOrUpdateSelection(target, event.all_intersecting_nodes);
      },
      this);

  // Handle when a model is loaded.
  editor_dispatcher.Connect(
      [this, &path_manager](const editor::ModelLoadedEvent& event) mutable {
        if (disable_select_model_on_load_) {
          return;
        }
        if (event.model->GetComponent<GltfRenderer>()) {
          // If a glTF was loaded, try to pass the first Collider in it we find
          // to get consistent behavior as if it was tapped.
          std::vector<ComponentHandle<GltfCollider>> colliders =
              path_manager.GetComponentsInDescendantsOrSelf<GltfCollider>(
                  event.model);
          if (!colliders.empty()) {
            NodeHandle collider_node = colliders.front()->GetNode();
            CycleOrUpdateSelection(collider_node, {collider_node});
            return;
          }
        }

        // Fallback to selecting the model itself.
        CycleOrUpdateSelection(event.model, {event.model});
      },
      this);

  // Move camera view to frame a selected node.
  editor_dispatcher.Connect(
      [this](const imp::KeyboardEvent& event) {
        if (!selected_node_.IsValid()) return;
        if (event.type == KeyboardEventType::kOnUp &&
            event.key.code == VirtualKeyCode::VK_f) {
          FocusCameraOnSelection(selected_node_);
        }
      },
      this);

  // Catches an event to toggle framing of a selected node.
  editor_dispatcher.Connect(
      [this](const FocusOnSelectionEvent& event) {
        FocusCameraOnSelection(event.node);
      },
      this);
}

NodeHandle SelectionControllerImpl::GetSelectedNode() const {
  return selected_node_;
}

bool SelectionControllerImpl::IsNodeOrAncestorSelected(NodeHandle node) {
  return selected_node_.IsValid() &&
         GetView().GetPathManager().IsAncestorOf(selected_node_, node);
}

void SelectionControllerImpl::TrySelectNode(NodeHandle node_to_select) {
  if (selected_node_ == node_to_select) {
    return;
  }

  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  Dispatcher& editor_dispatcher = editor.GetDispatcher();
  editor_dispatcher.Send(
      NodeSelectionChangedEvent(node_to_select, selected_node_));
  selected_node_ = node_to_select;
}

void SelectionControllerImpl::CycleOrUpdateSelection(
    NodeHandle target, std::vector<NodeHandle> intersecting_nodes) {
  PathManager& path_manager = GetView().GetPathManager();

  // Build a vector of nodes to cycle through.
  std::vector<NodeHandle> new_selectable_nodes;
  // Quick look up for ancestor nodes.
  absl::flat_hash_set<NodeHandle> gltf_renderer_nodes;
  for (const auto& selectable_node : intersecting_nodes) {
    auto gltf_renderer =
        path_manager.GetComponentFromAncestorOrSelf<GltfRenderer>(
            selectable_node);
    // Add the ancestor node to selectable nodes, if not already seen.
    if (gltf_renderer &&
        !gltf_renderer_nodes.contains(gltf_renderer->GetNode())) {
      NodeHandle gltf_renderer_node = gltf_renderer->GetNode();
      new_selectable_nodes.push_back(gltf_renderer_node);
      gltf_renderer_nodes.insert(gltf_renderer_node);
    }
    // Add the target node to selectable nodes.
    new_selectable_nodes.push_back(selectable_node);
  }

  // Selection logic: each subsequent tap selects one node deeper in the
  // selection list, as long as the list remains constant up to that
  // depth. Wraps around to front of the list otherwise.
  if (!new_selectable_nodes.empty()) {
    if (selectable_nodes_.empty()) {
      // On the first selection, pick the first node.
      selected_node_index_ = 0;
    } else {
      // On subsequent selections, try to select one node deeper.
      selected_node_index_ =
          (selected_node_index_ + 1) % new_selectable_nodes.size();
      // Check if all nodes match up to this depth.
      for (int i = 0; i <= selected_node_index_; ++i) {
        if (i >= selectable_nodes_.size() ||
            selectable_nodes_[i] != new_selectable_nodes[i]) {
          // If there is a mismatch, the selection is not constant at this
          // depth. So cycle back to 0.
          selected_node_index_ = 0;
          break;
        }
      }
    }
    target = new_selectable_nodes[selected_node_index_];
  }
  selectable_nodes_ = new_selectable_nodes;
  // TODO: Review this function to make sure the target exists.
  TrySelectNode(target);
}

void SelectionControllerImpl::FocusCameraOnSelection(NodeHandle target) {
  ComponentHandle<CameraComponent> editor_camera =
      GetView().GetRegistry().Get<Editor>()->get().GetCamera();

  absl::Status status = MoveIntoView(editor_camera, target,
                                     CameraHelperOptions::kIncludeDescendants);
  // If there was nothing viewable found in the selection,
  // the camera was not moved. Simply exit early from the function.
  if (!status.ok()) return;

  // Update the pivot.
  float3 cam_position = editor_camera->GetNode()->GetWorldPosition();
  NodeHandle pivot = editor_camera->GetNode()->GetParent();
  pivot->SetWorldPosition(target->GetWorldPosition());
  // After setting the pivot, we need to set the editor camera position again.
  editor_camera->GetNode()->SetWorldPosition(cam_position);
}

}  // namespace imp::editor
