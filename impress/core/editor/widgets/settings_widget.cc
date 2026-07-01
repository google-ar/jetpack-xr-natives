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

#include <optional>

#include "dear_imgui/imgui.h"
#include "filament/filament/include/filament/View.h"
#include "core/common/platform_storage.h"
#include "core/editor/components/grid.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/editor/widgets/asset_library.h"
#include "core/editor/widgets/settings_widget_constants.h"
#include "core/editor/widgets/visualize_bounds.h"
#include "core/editor/widgets/visualize_colliders.h"
#include "core/editor/widgets/visualize_origins.h"
#include "core/view/base_view.h"
#include "core/window/filament_host.h"

namespace imp::editor {

SettingsWidget::SettingsWidget(BaseView& view)
    : view_(view), editor_(*view_.GetRegistry().Get<editor::Editor>()) {
  PlatformStorage& storage = *view_.GetRegistry().Get<PlatformStorage>();
  grid_enabled_ = storage.GetBool(kGridEnabledKey, kGridEnabledDefault);
  skybox_enabled_ = storage.GetBool(kSkyboxEnabledKey, kSkyboxEnabledDefault);
  show_bounds_enabled_ =
      storage.GetBool(kShowBoundsEnabledKey, kShowBoundsEnabledDefault);
  show_all_colliders_enabled_ = storage.GetBool(
      kShowAllCollidersEnabledKey, kShowAllCollidersEnabledDefault);
  show_all_origins_enabled_ =
      storage.GetBool(kShowAllOriginsEnabledKey, kShowAllOriginsEnabledDefault);
  show_physics_visualizer_enabled_ = storage.GetBool(
      kShowPhysicsVisualizerEnabledKey, kShowPhysicsVisualizerEnabledDefault);
  vertex_selection_enabled_ = storage.GetBool(kVertexSelectionEnabledKey,
                                              kVertexSelectionEnabledDefault);
  load_mesh_data_on_cpu_enabled_ = storage.GetBool(
      kLoadMeshDataOnCpuEnabledKey, kLoadMeshDataOnCpuEnabledDefault);
  bvh_mesh_collision_acceleration_enabled_ =
      storage.GetBool(kBvhMeshCollisionAccelerationEnabledKey,
                      kBvhMeshCollisionAccelerationEnabledDefault);
}

void SettingsWidget::DrawImGui() {
  PlatformStorage& storage = *view_.GetRegistry().Get<PlatformStorage>();
  std::optional<editor::EditorSettingChangedEvent> event;
  if (ImGui::MenuItem(kGridText.data(), nullptr, &grid_enabled_)) {
    event.emplace().grid_enabled = grid_enabled_;
    storage.SetBool(kGridEnabledKey, grid_enabled_);
  }
  if (ImGui::MenuItem(kSkyboxText.data(), nullptr, &skybox_enabled_)) {
    event.emplace().skybox_enabled = skybox_enabled_;
    storage.SetBool(kSkyboxEnabledKey, skybox_enabled_);
  }
  if (ImGui::MenuItem(kShowBoundsText.data(), nullptr, &show_bounds_enabled_)) {
    event.emplace().show_all_bounds_enabled = show_bounds_enabled_;
    storage.SetBool(kShowBoundsEnabledKey, show_bounds_enabled_);
  }
  if (ImGui::MenuItem(kShowAllCollidersText.data(), nullptr,
                      &show_all_colliders_enabled_)) {
    event.emplace().show_all_colliders_enabled = show_all_colliders_enabled_;
    storage.SetBool(kShowAllCollidersEnabledKey, show_all_colliders_enabled_);
  }
  if (ImGui::MenuItem(kShowAllOriginsText.data(), nullptr,
                      &show_all_origins_enabled_)) {
    event.emplace().show_all_origins_enabled = show_all_origins_enabled_;
    storage.SetBool(kShowAllOriginsEnabledKey, show_all_origins_enabled_);
  }
  if (ImGui::MenuItem(kShowPhysicsVisualizerText.data(), nullptr,
                      &show_physics_visualizer_enabled_)) {
    event.emplace().show_physics_visualizer_enabled =
        show_physics_visualizer_enabled_;
    storage.SetBool(kShowPhysicsVisualizerEnabledKey,
                    show_physics_visualizer_enabled_);
  }
  if (ImGui::MenuItem(kEnableVertexSelection.data(), nullptr,
                      &vertex_selection_enabled_)) {
    event.emplace().vertex_selection_enabled = vertex_selection_enabled_;
    storage.SetBool(kVertexSelectionEnabledKey, vertex_selection_enabled_);
  }
  if (ImGui::MenuItem(kEnableLoadMeshDataOnCpu.data(), nullptr,
                      &load_mesh_data_on_cpu_enabled_)) {
    event.emplace().load_mesh_data_on_cpu_enabled =
        load_mesh_data_on_cpu_enabled_;
    bvh_mesh_collision_acceleration_enabled_ = false;
    storage.SetBool(kLoadMeshDataOnCpuEnabledKey,
                    load_mesh_data_on_cpu_enabled_);
    storage.SetBool(kBvhMeshCollisionAccelerationEnabledKey, false);
  }
  if (ImGui::MenuItem(kEnableBvhMeshCollisionAcceleration.data(), nullptr,
                      &bvh_mesh_collision_acceleration_enabled_)) {
    event.emplace().bvh_mesh_collision_acceleration_enabled =
        bvh_mesh_collision_acceleration_enabled_;
    load_mesh_data_on_cpu_enabled_ = true;
    storage.SetBool(kBvhMeshCollisionAccelerationEnabledKey,
                    bvh_mesh_collision_acceleration_enabled_);
    storage.SetBool(kLoadMeshDataOnCpuEnabledKey, true);
  }

  // If any settings changed, send the event.
  if (event) {
    editor_.GetDispatcher().Send(*event);
  }

  filament::View* filament_view = view_.GetHost()->GetView();
  bool is_post_processing_enabled = filament_view->isPostProcessingEnabled();
  if (ImGui::MenuItem(kEnablePostProcessingText.data(), nullptr,
                      &is_post_processing_enabled)) {
    filament_view->setPostProcessingEnabled(is_post_processing_enabled);
    storage.SetBool(kPostProcessingEnabledKey, is_post_processing_enabled);
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
