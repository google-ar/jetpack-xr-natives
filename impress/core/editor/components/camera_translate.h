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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_CAMERA_TRANSLATE_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_CAMERA_TRANSLATE_H_

#include <cstddef>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

#include "core/math/math.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

// Translates the camera on the x/z plane based on drag gestures.
class CameraTranslate : public Component {
 public:
  // Configures the component to translate the given pivot node.
  void Setup(NodeHandle pivot);

 private:
  std::optional<float3> GetPointerIntersectionWithHorizontalPivotPlane(
      float2 pointer);
  // Updates the pivot of the camera
  void MaybeUpdateCameraPivot(float2 position);

  std::optional<float3> intersection_prev_;
  NodeHandle pivot_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_CAMERA_TRANSLATE_H_
