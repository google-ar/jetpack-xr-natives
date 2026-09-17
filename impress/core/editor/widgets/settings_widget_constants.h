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

#ifndef THIRD_PARTY_IMPRESS_EDITOR_WIDGETS_SETTINGS_WIDGET_CONSTANTS_H_
#define THIRD_PARTY_IMPRESS_EDITOR_WIDGETS_SETTINGS_WIDGET_CONSTANTS_H_

#include "absl/strings/string_view.h"
namespace imp::editor {

constexpr absl::string_view kFileHeaderText = "File";
constexpr absl::string_view kSettingsText = "Settings";
constexpr absl::string_view kInputText = "Input";
constexpr absl::string_view kSkyboxText = "Show Skybox";
constexpr absl::string_view kShowBoundsText = "Show All Bounds";
constexpr absl::string_view kShowAllCollidersText = "Show All Colliders";
constexpr absl::string_view kShowAllOriginsText = "Show All Origins";
constexpr absl::string_view kShowPhysicsVisualizerText =
    "Show Physics Colliders & Constraints";
constexpr absl::string_view kEnablePostProcessingText =
    "Enable Post Processing";
constexpr absl::string_view kEnableVertexSelection = "Enable Vertex Selection";
constexpr absl::string_view kSaveAssetsToDisk = "Save Assets To Disk";
constexpr absl::string_view kEnableLoadMeshDataOnCpu =
    "Enable Load Mesh Data On CPU";
constexpr absl::string_view kEnableBvhMeshCollisionAcceleration =
    "Enable BVH Mesh Collision Acceleration";
constexpr absl::string_view kAutoSwitchToAppCameraOnPlayText =
    "Auto-switch to App Camera on Play";

inline constexpr absl::string_view kSkyboxEnabledKey = "editor_skybox_enabled";
inline constexpr bool kSkyboxEnabledDefault = true;

inline constexpr absl::string_view kAutoSwitchToAppCameraOnPlayEnabledKey =
    "editor_auto_switch_to_app_camera_on_play_enabled";
inline constexpr bool kAutoSwitchToAppCameraOnPlayEnabledDefault = true;

inline constexpr absl::string_view kVertexSelectionEnabledKey =
    "editor_vertex_selection_enabled";
inline constexpr bool kVertexSelectionEnabledDefault = false;

inline constexpr absl::string_view kTransformWidgetLocalSpaceKey =
    "editor_transform_widget_local_space";
inline constexpr bool kTransformWidgetLocalSpaceDefault = true;

inline constexpr absl::string_view kLoadMeshDataOnCpuEnabledKey =
    "editor_load_mesh_data_on_cpu_enabled";
inline constexpr bool kLoadMeshDataOnCpuEnabledDefault = false;

inline constexpr absl::string_view kBvhMeshCollisionAccelerationEnabledKey =
    "editor_bvh_mesh_collision_acceleration_enabled";
inline constexpr bool kBvhMeshCollisionAccelerationEnabledDefault = false;

inline constexpr absl::string_view kPostProcessingEnabledKey =
    "editor_post_processing_enabled";
inline constexpr bool kPostProcessingEnabledDefault = true;

inline constexpr absl::string_view kShowPhysicsVisualizerEnabledKey =
    "editor_show_physics_visualizer_enabled";
inline constexpr bool kShowPhysicsVisualizerEnabledDefault = false;

inline constexpr absl::string_view kSnapEnabledKey = "editor_snap_enabled";
inline constexpr bool kSnapEnabledDefault = false;

inline constexpr absl::string_view kGridSpacingKey = "editor_grid_spacing";
inline constexpr float kGridSpacingDefault = 1.0f;

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_EDITOR_WIDGETS_SETTINGS_WIDGET_CONSTANTS_H_
