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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_DESKTOP_CAMERA_CONTROLLER_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_DESKTOP_CAMERA_CONTROLLER_H_

#include "core/ar/ar_plane.h"
#include "core/common/debug_draw.h"
#include "core/view/framework/input/desktop_gesture_emulator.h"
#include "core/view/framework/input/desktop_input_handler.h"
#include "imp.h"

namespace imp {
namespace ar {
// AR Camera controller when running on desktop. Configures the virtual camera
// for simulated AR interaction.
class DesktopCameraController {
 public:
  DesktopCameraController(BaseView* view);
  void Update();
  void Initialize(bool load_virtual_environment);
  void Shutdown();
  void Resume();
  void Pause();
  quatf GetOrientation() { return orientation_; }
  mat4f GetViewMatrix();
  mat4 GetProjectionMatrix() { return projection_matrix_; }
  bool IsReadyToRender() const;

 private:
  void LoadDesktopEnvironment();

  // Sets up keyboard handling for toggling AR features.
  void ConfigureKeyboardInputListener();

  mat4 projection_matrix_;
  BaseView* view_;
  float2 last_pointer_;
  float2 pointer_;
  NodeHandle desktop_environment_;
  Dispatcher::Connection input_event_connection_;
  Dispatcher::Connection keyboard_event_connection_;
  Future<absl::Status> pending_model_;
  quatf orientation_;
  float3 position_;
  quatf yaw_;
  quatf pitch_;
  float3 move_vector_;
  std::unique_ptr<debug_draw::Fixture> debug_draw_fixture_ptr_;
  bool active_;
  DesktopGestureEmulator desktop_gesture_emulator_;
};
}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_DESKTOP_CAMERA_CONTROLLER_H_
