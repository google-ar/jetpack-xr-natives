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

#include <optional>

#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

// This forward declaration is required so that classes can reference
// NodeSelectionChangedEvent without needing to depend on SelectionController's
// implementation.
class SelectionControllerImpl;
// Event sent when a node is selected or deselected.
struct NodeSelectionChangedEvent : public Event {
 private:
  explicit NodeSelectionChangedEvent(NodeHandle selected_node,
                                     NodeHandle deselected_node)
      : selected(selected_node), deselected(deselected_node) {}

 public:
  NodeHandle selected;
  NodeHandle deselected;
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

// Event sent when a previewer setting is changed.
struct EditorSettingChangedEvent : public Event {
  std::optional<bool> grid_enabled;
  std::optional<bool> skybox_enabled;
  std::optional<bool> show_all_bounds_enabled;
  std::optional<bool> show_all_colliders_enabled;
  std::optional<bool> show_all_origins_enabled;
  std::optional<bool> vertex_selection_enabled;
  std::optional<bool> load_mesh_data_on_cpu_enabled;
  std::optional<bool> bvh_mesh_collision_acceleration_enabled;
};

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
struct FocusOnSelectionEvent : public Event {
  FocusOnSelectionEvent(NodeHandle node) : node(node) {}
  NodeHandle node;
};

// Event sent to notify the editor being enabled or disabled.
struct EditorEnabledEvent : public Event {
  explicit EditorEnabledEvent(bool editor_enabled) : enabled(editor_enabled) {}
  bool enabled;
};

// Event sent to enable or disable inverting camera Y.
struct InvertCameraYEvent : public Event {
  explicit InvertCameraYEvent(bool invert_camera_y_enabled)
      : enabled(invert_camera_y_enabled) {}
  bool enabled;
};

// Event sent to enable or disable inverting mouse scroll.
struct InvertMouseScrollEvent : public Event {
  explicit InvertMouseScrollEvent(bool invert_mouse_scroll_enabled)
      : enabled(invert_mouse_scroll_enabled) {}
  bool enabled;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EVENTS_H_
