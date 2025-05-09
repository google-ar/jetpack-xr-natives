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
#include "core/editor/xr/xr_grab_handle.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
namespace imp::editor {

// A Component that sets up the Impress Editor to render in world space for XR.
//
// A WorldSpaceEditorUi is positioned at a fixed distance from the camera. The
// Editor can be repositioned using a drag-and-drop handle that is rendered
// above the main Editor panel.
class XrEditorUi : public imp::Component {
 public:
  struct SetupOptions {
    // Defines the world-space distance the Impress Editor panel will be
    // rendered from the Camera.
    float distance_from_camera;

    // Scales the entire Editor panel up (maintaining the same
    // underlying resolution) by a given factor. This can be used if the UI is
    // too small to see.
    float editor_scale;

    // Degrees to the right of the horizontal center where the panel will be
    // anchored to. The center of the panel will be aligned to this angle.
    float horizontal_offset_degrees;

    // Degrees above the horizon where the Editor panel will be anchored to. The
    // top of the Editor panel will be aligned to this angle.
    float vertical_offset_degrees;

    // The underlying resolution of the ImGui UI. Increasing this without also
    // increasing the editor_scale will make the UI elements appear smaller.
    imp::float2 editor_resolution;
  };
  void Setup(const SetupOptions& options = SetupOptions{
                 .distance_from_camera = 3.0f,
                 .editor_scale = 2.0f,
                 .horizontal_offset_degrees = -45.0f,
                 .vertical_offset_degrees = 0.0f,
                 .editor_resolution = {500, 500}});

 private:
  imp::NodeHandle editor_node_;
  imp::ComponentHandle<XrGrabHandle> xr_editor_grab_handle_;
  imp::ComponentHandle<WorldSpaceEditorUi> world_space_editor_ui_;
  imp::float3 ray_origin_;
  imp::float3 ray_direction_;
  bool button_down_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_PROTOTYPES_SAMPLES_XR_EDITOR_SAMPLE_XR_EDITOR_UI_H_
