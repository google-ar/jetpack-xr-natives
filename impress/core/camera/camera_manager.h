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

#ifndef THIRD_PARTY_IMPRESS_CORE_CAMERA_CAMERA_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_CAMERA_CAMERA_MANAGER_H_

#include "core/camera/camera_component.h"
#include "core/ncsb/node.h"
#include "core/view/base_view.h"

namespace imp {

class CameraManager {
 public:
  explicit CameraManager(BaseView* view);

  // The first time this is called, creates the default camera and assigns
  // it as the currently used camera.
  void InitializeDefaultCamera();

  // Sets the camera that is used to render the view.
  //
  // If an invalid camera is passed in, then the camera is reset back to the
  // default camera returned by GetDefaultCamera().
  void SetCamera(ComponentHandle<CameraComponent> camera);

  // Returns the camera being used to render the view.
  //
  // Unless SetCamera is called, this is the same as GetDefaultCamera().
  ComponentHandle<CameraComponent> GetCamera() const;

  // Returns the camera used to render the view by default.
  ComponentHandle<CameraComponent> GetDefaultCamera() const;

 private:
  BaseView* view_;

  ComponentHandle<CameraComponent> default_camera_;
  ComponentHandle<CameraComponent> camera_;
  Dispatcher::ScopedConnection transition_parameters_changed_connection_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CAMERA_CAMERA_MANAGER_H_
