// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/scripting/message_handlers/set_camera_projection_handler.h"

#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/framework/camera/camera_manager.h"

namespace imp::scripting {

SetCameraProjectionHandler::SetCameraProjectionHandler(BaseView& view)
    : view_(view) {}

Future<absl::Status> SetCameraProjectionHandler::HandleMessage(
    const SetCameraProjectionRequest& message) {
  ComponentHandle<CameraComponent> camera =
      view_.GetCameraManager().GetCamera();
  if (message.near) {
    camera->SetNearClip(*message.near);
  }
  if (message.far) {
    camera->SetFarClip(*message.far);
  }
  if (message.matrix()) {
    camera->SetProjectionMatrix(*message.matrix());
  } else if (message.vertical_fov()) {
    camera->SetVerticalFovInDegrees(*message.vertical_fov());
  }

  return Future<absl::Status>(absl::OkStatus());
}

}  // namespace imp::scripting
