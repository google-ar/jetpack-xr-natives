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

#include "core/editor/widgets/settings_widget.h"

#include "dear_imgui/imgui.h"
#include "filament/filament/include/filament/View.h"
#include "core/common/platform_storage.h"
#include "core/editor/editor.h"
#include "core/editor/widgets/asset_library.h"
#include "core/editor/widgets/settings_widget_constants.h"
#include "core/view/base_view.h"
#include "core/window/filament_host.h"

namespace imp::editor {

SettingsWidget::SettingsWidget(BaseView& view)
    : view_(view), editor_(*view_.GetRegistry().Get<editor::Editor>()) {}

void SettingsWidget::DrawImGui() {
  PlatformStorage& storage = *view_.GetRegistry().Get<PlatformStorage>();
  bool skybox_enabled = editor_.GetSettings().skybox_enabled.Get();
  if (ImGui::MenuItem(kSkyboxText.data(), nullptr, &skybox_enabled)) {
    editor_.GetSettings().skybox_enabled.Set(skybox_enabled);
  }
  bool show_bounds_enabled =
      editor_.GetSettings().show_all_bounds_enabled.Get();
  if (ImGui::MenuItem(kShowBoundsText.data(), nullptr, &show_bounds_enabled)) {
    editor_.GetSettings().show_all_bounds_enabled.Set(show_bounds_enabled);
  }
  bool show_all_colliders =
      editor_.GetSettings().show_all_colliders_enabled.Get();
  if (ImGui::MenuItem(kShowAllCollidersText.data(), nullptr,
                      &show_all_colliders)) {
    editor_.GetSettings().show_all_colliders_enabled.Set(show_all_colliders);
  }
  bool show_all_origins = editor_.GetSettings().show_all_origins_enabled.Get();
  if (ImGui::MenuItem(kShowAllOriginsText.data(), nullptr, &show_all_origins)) {
    editor_.GetSettings().show_all_origins_enabled.Set(show_all_origins);
  }
  bool show_physics_visualizer =
      editor_.GetSettings().show_physics_visualizer_enabled.Get();
  if (ImGui::MenuItem(kShowPhysicsVisualizerText.data(), nullptr,
                      &show_physics_visualizer)) {
    editor_.GetSettings().show_physics_visualizer_enabled.Set(
        show_physics_visualizer);
  }
  bool vertex_selection = editor_.GetSettings().vertex_selection_enabled.Get();
  if (ImGui::MenuItem(kEnableVertexSelection.data(), nullptr,
                      &vertex_selection)) {
    editor_.GetSettings().vertex_selection_enabled.Set(vertex_selection);
  }
  bool load_mesh = editor_.GetSettings().load_mesh_data_on_cpu_enabled.Get();
  if (ImGui::MenuItem(kEnableLoadMeshDataOnCpu.data(), nullptr, &load_mesh)) {
    editor_.GetSettings().load_mesh_data_on_cpu_enabled.Set(load_mesh);
    editor_.GetSettings().bvh_mesh_collision_acceleration_enabled.Set(false);
  }
  bool bvh_acceleration =
      editor_.GetSettings().bvh_mesh_collision_acceleration_enabled.Get();
  if (ImGui::MenuItem(kEnableBvhMeshCollisionAcceleration.data(), nullptr,
                      &bvh_acceleration)) {
    editor_.GetSettings().bvh_mesh_collision_acceleration_enabled.Set(
        bvh_acceleration);
    editor_.GetSettings().load_mesh_data_on_cpu_enabled.Set(true);
  }

  filament::View* filament_view = view_.GetHost()->GetView();
  bool is_post_processing_enabled = filament_view->isPostProcessingEnabled();
  if (ImGui::MenuItem(kEnablePostProcessingText.data(), nullptr,
                      &is_post_processing_enabled)) {
    filament_view->setPostProcessingEnabled(is_post_processing_enabled);
    storage.SetBool(kPostProcessingEnabledKey, is_post_processing_enabled);
  }

  if (editor_.IsSandbox()) {
    bool auto_switch =
        editor_.GetSettings().auto_switch_to_app_camera_on_play.Get();
    if (ImGui::MenuItem(kAutoSwitchToAppCameraOnPlayText.data(), nullptr,
                        &auto_switch)) {
      editor_.GetSettings().auto_switch_to_app_camera_on_play.Set(auto_switch);
    }
  }

  // Saving assets is only supported on desktop, so only show the option on
  // desktop.
  if constexpr (IMP_PLATFORM(DESKTOP)) {
    // Toggle to control if assets are saved to disk. If false, assets are just
    // stored virtually in the asset library.
    bool is_save_assets_to_disk_enabled =
        editor_.GetAssetLibrary()->IsSavingToDiskEnabled();
    if (ImGui::MenuItem(kSaveAssetsToDisk.data(), nullptr,
                        &is_save_assets_to_disk_enabled)) {
      editor_.GetAssetLibrary()->SetSavingToDiskEnabled(
          is_save_assets_to_disk_enabled);
    }
  }
}
}  // namespace imp::editor
