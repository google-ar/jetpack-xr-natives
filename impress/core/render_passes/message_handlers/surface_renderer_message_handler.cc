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

#include "core/render_passes/message_handlers/surface_renderer_message_handler.h"

#include "absl/status/status.h"
#include "core/common/platform_helpers.h"

namespace imp {

Future<absl::Status> SurfaceRendererMessageHandler::HandleMessage(
    const SetCameraRequest& message) {
  if (!message.surface_renderer_node) {
    return Future<absl::Status>(absl::InvalidArgumentError(
        "SetCameraRequest: SurfaceRenderer node handle is invalid."));
  }

  ComponentHandle<SurfaceRenderer> surface_renderer =
      message.surface_renderer_node->GetComponent<SurfaceRenderer>();

  if (!surface_renderer) {
    return Future<absl::Status>(
        absl::InternalError("SetCameraRequest: Failed to find SurfaceRenderer "
                            "on the target node."));
  }

  if (message.camera_node_name()) {
    return Future<absl::Status>(
        surface_renderer->SetCamera(*message.camera_node_name()));
  } else if (message.camera_node()) {
    surface_renderer->SetCamera(
        (*message.camera_node())->GetComponent<CameraComponent>());
    return Future<absl::Status>(absl::OkStatus());
  } else {
    surface_renderer->SetCamera(view_->GetCameraManager().GetCamera());
    return Future<absl::Status>(absl::OkStatus());
  }
}

Future<absl::Status> SurfaceRendererMessageHandler::HandleMessage(
    const SetGroupRequest& message) {
  if (!message.surface_renderer_node) {
    return Future<absl::Status>(absl::InvalidArgumentError(
        "SetCameraRequest: SurfaceRenderer node handle is invalid."));
  }

  ComponentHandle<SurfaceRenderer> surface_renderer =
      message.surface_renderer_node->GetComponent<SurfaceRenderer>();

  if (!surface_renderer) {
    return Future<absl::Status>(
        absl::InternalError("SetCameraRequest: Failed to find SurfaceRenderer "
                            "on the target node."));
  }

  surface_renderer->SetGroup(message.group);
  return Future<absl::Status>(absl::OkStatus());
}

Future<absl::Status> SurfaceRendererMessageHandler::HandleMessage(
    const SetViewPortSizeRequest& message) {
  if (!message.surface_renderer_node) {
    return Future<absl::Status>(absl::InvalidArgumentError(
        "SetViewPortSizeRequest: SurfaceRenderer node handle is invalid."));
  }

  ComponentHandle<SurfaceRenderer> surface_renderer =
      message.surface_renderer_node->GetComponent<SurfaceRenderer>();

  if (!surface_renderer) {
    return Future<absl::Status>(absl::InternalError(
        "SetViewPortSizeRequest: Failed to find SurfaceRenderer "
        "on the target node."));
  }

  surface_renderer->SetViewPortSize(message.view_port_size);

  return Future<absl::Status>(absl::OkStatus());
}

}  // namespace imp
