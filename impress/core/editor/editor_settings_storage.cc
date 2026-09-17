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

#include "core/editor/editor_settings_storage.h"

#include <string>

#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "core/common/platform_storage.h"
#include "core/editor/common/notifying_property.h"
#include "core/editor/components/grid.h"
#include "core/editor/editor_settings.h"
#include "core/editor/widgets/input_settings_widget_constants.h"
#include "core/editor/widgets/settings_widget_constants.h"
#include "core/editor/widgets/visualize_bounds.h"
#include "core/editor/widgets/visualize_colliders.h"
#include "core/editor/widgets/visualize_origins.h"

namespace imp::editor {

EditorSettingsStorage::EditorSettingsStorage(PlatformStorage& storage,
                                             EditorSettings& settings)
    : storage_(storage) {
  // Load initial settings.
  settings.show_grid = Load<bool>(kGridEnabledKey, kGridEnabledDefault);
  settings.use_legacy_camera_controls =
      Load<bool>(kUseLegacyCameraControlsKey, kUseLegacyCameraControlsDefault);
  settings.skybox_enabled =
      Load<bool>(kSkyboxEnabledKey, kSkyboxEnabledDefault);
  settings.show_all_bounds_enabled =
      Load<bool>(kShowBoundsEnabledKey, kShowBoundsEnabledDefault);
  settings.show_all_colliders_enabled =
      Load<bool>(kShowAllCollidersEnabledKey, kShowAllCollidersEnabledDefault);
  settings.show_all_origins_enabled =
      Load<bool>(kShowAllOriginsEnabledKey, kShowAllOriginsEnabledDefault);
  settings.show_physics_visualizer_enabled = Load<bool>(
      kShowPhysicsVisualizerEnabledKey, kShowPhysicsVisualizerEnabledDefault);
  settings.vertex_selection_enabled =
      Load<bool>(kVertexSelectionEnabledKey, kVertexSelectionEnabledDefault);
  settings.load_mesh_data_on_cpu_enabled = Load<bool>(
      kLoadMeshDataOnCpuEnabledKey, kLoadMeshDataOnCpuEnabledDefault);
  settings.bvh_mesh_collision_acceleration_enabled =
      Load<bool>(kBvhMeshCollisionAccelerationEnabledKey,
                 kBvhMeshCollisionAccelerationEnabledDefault);
  settings.invert_camera_y_enabled =
      Load<bool>(kInvertCameraYKey, kInvertCameraYDefault);
  settings.invert_mouse_scroll_enabled =
      Load<bool>(kInvertMouseScrollKey, kInvertMouseScrollDefault);
  settings.auto_switch_to_app_camera_on_play =
      Load<bool>(kAutoSwitchToAppCameraOnPlayEnabledKey,
                 kAutoSwitchToAppCameraOnPlayEnabledDefault);
  settings.transform_widget_local_space = Load<bool>(
      kTransformWidgetLocalSpaceKey, kTransformWidgetLocalSpaceDefault);
  settings.snap_enabled = Load<bool>(kSnapEnabledKey, kSnapEnabledDefault);
  settings.grid_spacing = Load<float>(kGridSpacingKey, kGridSpacingDefault);

  // Bind notify properties to automatically update storage when they change.
  show_grid_binding_ = Bind<bool>(settings.show_grid, kGridEnabledKey);

  use_legacy_camera_controls_binding_ = Bind<bool>(
      settings.use_legacy_camera_controls, kUseLegacyCameraControlsKey);

  skybox_binding_ = Bind<bool>(settings.skybox_enabled, kSkyboxEnabledKey);

  show_all_bounds_binding_ =
      Bind<bool>(settings.show_all_bounds_enabled, kShowBoundsEnabledKey);

  show_all_colliders_binding_ = Bind<bool>(settings.show_all_colliders_enabled,
                                           kShowAllCollidersEnabledKey);

  show_all_origins_binding_ =
      Bind<bool>(settings.show_all_origins_enabled, kShowAllOriginsEnabledKey);

  show_physics_visualizer_binding_ =
      Bind<bool>(settings.show_physics_visualizer_enabled,
                 kShowPhysicsVisualizerEnabledKey);

  vertex_selection_binding_ =
      Bind<bool>(settings.vertex_selection_enabled, kVertexSelectionEnabledKey);

  load_mesh_data_on_cpu_binding_ = Bind<bool>(
      settings.load_mesh_data_on_cpu_enabled, kLoadMeshDataOnCpuEnabledKey);

  bvh_mesh_collision_acceleration_binding_ =
      Bind<bool>(settings.bvh_mesh_collision_acceleration_enabled,
                 kBvhMeshCollisionAccelerationEnabledKey);

  invert_camera_y_binding_ =
      Bind<bool>(settings.invert_camera_y_enabled, kInvertCameraYKey);

  invert_mouse_scroll_binding_ =
      Bind<bool>(settings.invert_mouse_scroll_enabled, kInvertMouseScrollKey);

  auto_switch_to_app_camera_on_play_binding_ =
      Bind<bool>(settings.auto_switch_to_app_camera_on_play,
                 kAutoSwitchToAppCameraOnPlayEnabledKey);

  transform_widget_local_space_binding_ = Bind<bool>(
      settings.transform_widget_local_space, kTransformWidgetLocalSpaceKey);
  snap_enabled_binding_ = Bind<bool>(settings.snap_enabled, kSnapEnabledKey);
  grid_spacing_binding_ = Bind<float>(settings.grid_spacing, kGridSpacingKey);
}

bool EditorSettingsStorage::Load(const absl::string_view key,
                                 const bool default_value) {
  return storage_.GetBool(key, default_value);
}

float EditorSettingsStorage::Load(const absl::string_view key,
                                  const float default_value) {
  const std::string val = storage_.GetString(key);

  if (val.empty()) return default_value;

  float result = default_value;

  if (!absl::SimpleAtof(val, &result)) return default_value;

  return result;
}

PropertyBinding<bool> EditorSettingsStorage::Bind(
    NotifyingProperty<bool>& property, const absl::string_view key) {
  const auto callback = [this, key](const bool value, bool) {
    storage_.SetBool(std::string(key), value);
  };
  return PropertyBinding<bool>(property, callback);
}

PropertyBinding<float> EditorSettingsStorage::Bind(
    NotifyingProperty<float>& property, const absl::string_view key) {
  const auto callback = [this, key](const float value, float) {
    storage_.SetString(std::string(key), absl::StrCat(value));
  };
  return PropertyBinding<float>(property, callback);
}

}  // namespace imp::editor
