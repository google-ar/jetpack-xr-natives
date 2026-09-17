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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_SETTINGS_STORAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_SETTINGS_STORAGE_H_

#include "absl/strings/string_view.h"
#include "core/common/platform_storage.h"
#include "core/editor/common/notifying_property.h"
#include "core/editor/editor_settings.h"

namespace imp::editor {

// Manages binding EditorSettings to PlatformStorage.
// On creation, load initial values from PlatformStorage into EditorSettings,
// then bind each setting to automatically save updates to PlatformStorage.
class EditorSettingsStorage {
 public:
  EditorSettingsStorage(PlatformStorage& storage, EditorSettings& settings);

 private:
  template <typename T>
  T Load(absl::string_view key, T default_value) {
    return Load(key, default_value);
  }

  template <typename T>
  [[nodiscard]] PropertyBinding<T> Bind(NotifyingProperty<T>& property,
                                        absl::string_view key) {
    return Bind(property, key);
  }

  bool Load(absl::string_view key, bool default_value);
  float Load(absl::string_view key, float default_value);

  [[nodiscard]] PropertyBinding<bool> Bind(NotifyingProperty<bool>& property,
                                           absl::string_view key);
  [[nodiscard]] PropertyBinding<float> Bind(NotifyingProperty<float>& property,
                                            absl::string_view key);

  PlatformStorage& storage_;

  PropertyBinding<bool> show_grid_binding_;
  PropertyBinding<bool> use_legacy_camera_controls_binding_;
  PropertyBinding<bool> skybox_binding_;
  PropertyBinding<bool> show_all_bounds_binding_;
  PropertyBinding<bool> show_all_colliders_binding_;
  PropertyBinding<bool> show_all_origins_binding_;
  PropertyBinding<bool> show_physics_visualizer_binding_;
  PropertyBinding<bool> vertex_selection_binding_;
  PropertyBinding<bool> load_mesh_data_on_cpu_binding_;
  PropertyBinding<bool> bvh_mesh_collision_acceleration_binding_;
  PropertyBinding<bool> invert_camera_y_binding_;
  PropertyBinding<bool> invert_mouse_scroll_binding_;
  PropertyBinding<bool> auto_switch_to_app_camera_on_play_binding_;
  PropertyBinding<bool> transform_widget_local_space_binding_;
  PropertyBinding<bool> snap_enabled_binding_;
  PropertyBinding<float> grid_spacing_binding_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_SETTINGS_STORAGE_H_
