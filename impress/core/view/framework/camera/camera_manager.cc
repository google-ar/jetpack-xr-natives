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

#include "core/view/framework/camera/camera_manager.h"

#include "core/view/view_events.h"

namespace imp {

static constexpr absl::string_view kCameraNodeName = "camera";

CameraManager::CameraManager(BaseView* view) : view_(view) {}

void CameraManager::InitializeDefaultCamera() {
  if (default_camera_) {
    return;
  }

  NodeHandle camera_node = view_->CreateNode();
  camera_node->SetName(kCameraNodeName);
  default_camera_ = camera_node->AddComponent<CameraComponent>();
  SetCamera(default_camera_);

  transition_parameters_changed_connection_ = view_->GetDispatcher().Connect(
      [this](const ViewTransitionParametersChangedEvent& ev) {
        auto cam = GetCamera();
        cam->HandleTransitionScale(ev.size_scale);
      });
}

void CameraManager::SetCamera(ComponentHandle<CameraComponent> camera) {
  if (!camera) {
    camera = default_camera_;
  }

  view_->GetHost()->GetView()->setCamera(camera->GetCamera());
  camera_ = camera;
}

ComponentHandle<CameraComponent> CameraManager::GetCamera() const {
  return camera_;
}

ComponentHandle<CameraComponent> CameraManager::GetDefaultCamera() const {
  return default_camera_;
}

}  // namespace imp
