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

#include <vector>

#include "core/config.h"
#include "core/input/keyboard_event.h"
#include "core/input/pointer_event.h"
#include "core/math/math.h"
#include "core/ncsb/component.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

// Provides camera rotation by dragging to view model. This is accomplished by
// setting a pivot where the model is, parenting the camera to the pivot, and
// then rotating the pivot according to the corresponding drag gesture.
// Additionally, this component provides free-cam movement when WASD keys are
// pressed and the right mouse button is held.
class CameraRotate : public Component {
 public:
  void Setup(NodeHandle pivot, float pitch, float yaw);
  void Update(const FrameTime& frame_time);

  // Called by the FreecamInterceptor
  void FilterPointerEvents(std::vector<PointerEvent>& pointer_events);
  void FilterKeyboardEvents(std::vector<KeyboardEvent>& keyboard_events);

 private:
  // Handles the rotation of the pivot based on mouse input.
  void HandleRotation();
  // Handles the movement of the pivot based on keyboard input.
  void HandleMovement(const FrameTime& frame_time);

  float pitch_;
  float yaw_;
  NodeHandle pivot_;
  bool invert_y_enabled_ = IMP_INVERT_EDITOR_INPUT_DEFAULT_VALUE;

  bool w_pressed_ = false;
  bool a_pressed_ = false;
  bool s_pressed_ = false;
  bool d_pressed_ = false;
  bool q_pressed_ = false;
  bool e_pressed_ = false;
  bool shift_pressed_ = false;
  bool camera_rotate_enabled_ = false;
  float2 rotation_delta_ = kZero2;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_CAMERA_ROTATE_H_
