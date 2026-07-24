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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_EVENTS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_EVENTS_H_

#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

// This forward declaration is required so that classes can reference
// NodeSelectionChangedEvent without needing to depend on SelectionController's
// implementation.
class SelectionControllerImpl;
// Event dispatched when the set of selected nodes changes. Listeners can query
// the Editor to retrieve the current selection. For example:
//
//   editor->GetEventBus()->AddListener<NodeSelectionChangedEvent>(
//       [editor](const NodeSelectionChangedEvent& event) {
//         const auto& selected_nodes =
//             editor->GetSelectionController().GetSelectedNodes();
//         for (NodeHandle selected_node : selected_nodes) {
//           // Do something with `selected_node`.
//         }
//       });
struct NodeSelectionChangedEvent : public Event {
 public:
  friend class SelectionControllerImpl;
};

// Event sent when model has begun loading in the asset manager.
struct ModelLoadingEvent : public Event {};

// Event sent when model has finished loading. The event sends the newly loaded
// NodeHandle.
struct ModelLoadedEvent : public Event {
  explicit ModelLoadedEvent(NodeHandle model_loaded) : model(model_loaded) {}
  NodeHandle model;
};

// Event sent when a node is clicked on using pixel-perfect filament pick.
struct PixelPickEvent : public Event {};

// Event that tells the editor to recreate all the component details widgets
// for the selected node. Call this when you add/remove a component from the
// selected node.
struct RefreshComponentWidgetsEvent : public Event {};

// Event sent to tell the editor to toggle between the editor/app camera.
struct ToggleCameraEvent : public Event {
  ToggleCameraEvent() = default;
};

// Event sent to tell the editor to move the camera view to focus on
// the current active node.
struct FocusOnSelectionEvent : public Event {};

// Event sent to notify the editor being enabled or disabled.
struct EditorEnabledEvent : public Event {
  explicit EditorEnabledEvent(bool editor_enabled) : enabled(editor_enabled) {}
  bool enabled;
};

// Event sent when local/worldspace mode of the transform widget is toggled.
struct TransformWidgetSpaceModeChangedEvent : public Event {
  explicit TransformWidgetSpaceModeChangedEvent(bool local_space_enabled)
      : local_space(local_space_enabled) {}
  bool local_space;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EVENTS_H_
