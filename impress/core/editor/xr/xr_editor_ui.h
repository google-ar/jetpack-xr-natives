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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_XR_EDITOR_UI_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_XR_EDITOR_UI_H_

#include "core/editor/components/world_space_editor_ui.h"
#include "core/editor/layout/editor_panel_ids.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/string_map.h"
namespace imp::editor {

// A Component that sets up the Impress Editor to render in world space for XR.
//
// A WorldSpaceEditorUi is positioned at a fixed distance from the camera. The
// Editor can be repositioned using a drag-and-drop handle that is rendered
// above the main Editor panel.
class XrEditorUi : public imp::Component {
 public:
  struct SetupOptions {
    // Scales the entire Editor panel up (maintaining the same
    // underlying resolution) by a given factor. This can be used if the UI is
    // too small to see.
    float editor_scale;

    // The underlying resolution of the ImGui UI. Increasing this without also
    // increasing the editor_scale will make the UI elements appear smaller.
    imp::float2 editor_resolution;

    // A map of editor panel names to their settings. If a panel is not
    // specified, it will default to WorldSpaceEditorUi::kDefaultCanvasPosition.
    //   - x(distance_from_camera)
    //   - y(horizontal_offset_degrees)
    //   - z(vertical_offset_degrees)
    StringMap<float3> editor_panel_settings;
  };
  void Setup(
      const SetupOptions& options = SetupOptions{
          .editor_scale = 2.0f,
          .editor_resolution = {2000, 800},
          .editor_panel_settings = {
              {panel_ids::kSceneWindow.data(), float3{3.0f, -55.0f, 0.0f}},
              {panel_ids::kDetailsWindow.data(), float3{3.0f, -30.0f, -10.0f}},
              {panel_ids::kTabBar.data(), float3{3.0f, -55.0f, 20.0f}}}});

 private:
  imp::NodeHandle editor_node_;
  imp::ComponentHandle<WorldSpaceEditorUi> world_space_editor_ui_;
  imp::float3 ray_origin_;
  imp::float3 ray_direction_;
  bool button_down_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_PROTOTYPES_SAMPLES_XR_EDITOR_SAMPLE_XR_EDITOR_UI_H_
