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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_CAMERA_ZOOM_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_CAMERA_ZOOM_H_

#include "core/config.h"
#include "core/math/math.h"
#include "core/ncsb/component.h"

namespace imp::editor {

// Zooms into the current active selection. Reads the mouse wheel delta values
// to determine which direction to zoom in.
class CameraZoom : public Component {
 public:
  void Setup(NodeHandle pivot);

 private:
  void UpdateCameraZoom(float zoom_delta, float zoom_sensitivity,
                        float zoom_min_distance);

  NodeHandle pivot_;
  float3 target_position_;
  bool invert_scroll_enabled_ = IMP_INVERT_EDITOR_INPUT_DEFAULT_VALUE;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_CAMERA_ZOOM_H_
