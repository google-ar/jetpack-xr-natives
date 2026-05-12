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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_SPATIAL_UI_CANVAS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_SPATIAL_UI_CANVAS_H_

#include <memory>
#include <string>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/ncsb/component.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

// A component that manages the correspondence between the an ImGui window and
// its corresponding panel in the world space, in terms of display and input.
class SpatialUiCanvas : public Component {
 public:
  // Information about the ImGui window and the panel it corresponds to in the
  // world space.
  struct SpatialUiCanvasSettings {
    // ImGui window label.
    std::string name;
    // ImGui window position. (upper left corner, in pixels)
    float2 content_position;
    // Imgui window size. (in pixels)
    float2 content_size;
  };

  Future<absl::Status> Setup(absl::string_view name, float2 content_position,
                             float2 content_size, BorrowedTexturePtr texture,
                             int2 texture_resolution);

  void Update(FrameTime& frame_time);

  // Updates the canvas with the new settings, in case the size or position of
  // the ImGui window has changed.
  void UpdateCanvas(SpatialUiCanvasSettings settings);

  // Returns the settings for the canvas.
  SpatialUiCanvasSettings GetSettings();

  // Transforms a world point into a pixel coordinate for the ImGui UI.
  ImVec2 CalculateImGuiPointFromWorldPoint(float3 world_point);

 private:
  BorrowedTexturePtr texture_;
  std::string name_;
  float2 content_position_;
  float2 content_size_;
  OwnedMaterialPtr material_;
  OwnedMeshPtr quad_mesh_;
  float2 global_texture_resolution_;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_SPATIAL_UI_CANVAS_H_
