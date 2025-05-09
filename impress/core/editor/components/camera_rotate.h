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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_CAMERA_ROTATE_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_CAMERA_ROTATE_H_

#include <cstddef>
#include <iostream>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/math/math.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

// Provides camera rotation by dragging to view model. This is accomplished by
// setting a pivot where the model is, parenting the camera to the pivot, and
// then rotating the pivot according to the corresponding drag gesture.
class CameraRotate : public Component {
 public:
  void Setup(NodeHandle pivot, float pitch, float yaw);

 private:
  float pitch_;
  float yaw_;
  NodeHandle pivot_;
  bool invert_y_enabled_ = IMP_INVERT_EDITOR_INPUT_DEFAULT_VALUE;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_CAMERA_ROTATE_H_
