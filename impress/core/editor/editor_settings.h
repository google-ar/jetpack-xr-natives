/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_SETTINGS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_SETTINGS_H_

#include "core/editor/common/notifying_property.h"

namespace imp::editor {

// User-configurable settings for the editor.
// Provides a single source of truth for the editor's user settings.
struct EditorSettings {
  NotifyingProperty<bool> show_grid{true};
  NotifyingProperty<bool> use_legacy_camera_controls{true};
  NotifyingProperty<bool> skybox_enabled{true};
  NotifyingProperty<bool> show_all_bounds_enabled{false};
  NotifyingProperty<bool> show_all_colliders_enabled{false};
  NotifyingProperty<bool> show_all_origins_enabled{false};
  NotifyingProperty<bool> show_physics_visualizer_enabled{false};
  NotifyingProperty<bool> vertex_selection_enabled{false};
  NotifyingProperty<bool> load_mesh_data_on_cpu_enabled{false};
  NotifyingProperty<bool> bvh_mesh_collision_acceleration_enabled{false};
  NotifyingProperty<bool> invert_camera_y_enabled{false};
  NotifyingProperty<bool> invert_mouse_scroll_enabled{false};
  NotifyingProperty<float> transform_widget_scale{1.0f};
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_SETTINGS_H_
