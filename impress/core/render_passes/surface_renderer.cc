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

#include "core/render_passes/surface_renderer.h"

#include <optional>
#include <string>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Viewport.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/utils/render_setting_utils.h"
#include "core/view/view_events.h"

namespace imp {

constexpr int64_t kSwapChainFlagTransparent = 0x1;

absl::Status SurfaceRenderer::Setup(
    void* native_window, std::optional<absl::string_view> group,
    std::optional<absl::string_view> camera_name) {
  MP_RETURN_IF_ERROR(
      Setup(native_window, group, GetView().GetCameraManager().GetCamera()));
  if (camera_name.has_value()) {
    MP_RETURN_IF_ERROR(SetCamera(camera_name.value()));
  }
  return absl::OkStatus();
}

absl::Status SurfaceRenderer::Setup(void* native_window,
                                    std::optional<absl::string_view> group) {
  return Setup(native_window, group, GetView().GetCameraManager().GetCamera());
}

absl::Status SurfaceRenderer::Setup(void* native_window,
                                    std::optional<absl::string_view> group,
                                    ComponentHandle<CameraComponent> camera) {
  SurfaceRendererSettings settings;
  if (group.has_value()) {
    settings.group = std::string(group.value());
  } else {
    settings.group = std::nullopt;
  }
  *settings.mutable_camera_node() = camera->GetNode();
  settings.render_settings = std::nullopt;
  return Setup(native_window, settings);
}

absl::Status SurfaceRenderer::Setup(void* native_window,
                                    const SurfaceRendererSettings& settings) {
  filament::Engine* engine = BaseView::GetSharedEngine();
  filament::View* host_view = GetView().GetHost()->GetView();

  // Need to create a dedicated filament::View for a SurfaceRenderer instance.
  filament_view_ = engine->createView();

  // Configure render settings.
  if (settings.render_settings.has_value()) {
    ConfigureViewRenderSettingsWithOverrides(
        settings.override_mode, filament_view_, host_view,
        &(*settings.render_settings), engine);
  } else {
    ConfigureViewRenderSettingsWithOverrides(
        settings.override_mode, filament_view_, host_view, nullptr, engine);
  }

  // Set up group
  if (settings.group.has_value()) {
    SetGroup(*settings.group);
  } else {
    SetGroup(GroupsManager::kMainGroupName);
  }

  // Set up camera.
  if (settings.camera_node()) {
    MP_RETURN_IF_ERROR(SetCamera((*settings.camera_node())->GetName()));
  } else if (settings.camera_node_name()) {
    MP_RETURN_IF_ERROR(SetCamera(*settings.camera_node_name()));
  } else {
    SetCamera(GetView().GetCameraManager().GetCamera());
  }

  // Set up viewport size.
  if (settings.view_port_size.has_value()) {
    SetViewPortSize(settings.view_port_size.value());
  } else {
    imp::float2 viewport_size =
        GetView().GetDevice().PixelsToPhysicalPixels(GetView().GetSize());
    filament_view_->setViewport({0, 0, static_cast<uint32_t>(viewport_size.x),
                                 static_cast<uint32_t>(viewport_size.y)});
  }

  display_id_ = settings.display_id;

  // Set up the actual swap chain for rendering.
  swap_chain_ = engine->createSwapChain(
      native_window,
      settings.swap_chain_flags.value_or(kSwapChainFlagTransparent));
  if (!swap_chain_) {
    return absl::InternalError(
        "Failed to create filament swap chain from native window.");
  }

  // Connect to the post-frame event so we render after the main scene.
  imp::Dispatcher& dispatcher = GetView().GetDispatcher();
  dispatcher.Connect(
      [this](const ViewSecondaryRenderEvent& secondary_render_event) {
        if (!IsActive() || !render_this_frame_) {
          return;
        }

        Render();
        render_this_frame_ = false;
      },
      this);

  return absl::OkStatus();
}

void SurfaceRenderer::Update(const imp::FrameTime& frame_time) {
  render_this_frame_ = true;
}

void SurfaceRenderer::Cleanup() {
  if (swap_chain_) {
    GetView().GetSharedEngine()->destroy(swap_chain_);
  }
  if (filament_view_) {
    GetView().GetSharedEngine()->destroy(filament_view_);
  }
}

absl::Status SurfaceRenderer::SetCamera(absl::string_view camera_node_name) {
  ComponentHandle<CameraComponent> camera_comp =
      FindCameraWithNodeName(camera_node_name);

  if (camera_comp) {
    SetCamera(camera_comp);
  } else {
    return absl::NotFoundError(absl::StrFormat(
        "SurfaceRenderer::Setup: Unable to find any CameraComponent in "
        "nodes with the name: %s",
        camera_node_name));
  }

  return absl::OkStatus();
}

void SurfaceRenderer::SetCamera(ComponentHandle<CameraComponent> camera) {
  if (!camera) {
    IMP_LOG(imp::FATAL) << "Invalid ComponentHandle<CameraComponent>.";
  } else {
    filament_view_->setCamera(camera->GetCamera());
    camera_ = camera;
  }
}

void SurfaceRenderer::SetGroup(absl::string_view group) {
  group_ = std::string(group);
}

void SurfaceRenderer::SetViewPortSize(uint2 view_port_size) {
  view_port_size_ = view_port_size;
  filament_view_->setViewport(
      {0, 0, (*view_port_size_).x, (*view_port_size_).y});
  Send(ViewPortSizeChangedEvent{GetViewPortSize()});
}

ComponentHandle<CameraComponent> SurfaceRenderer::GetCamera() const {
  return camera_;
}

absl::string_view SurfaceRenderer::GetGroup() const { return group_; }

imp::uint2 SurfaceRenderer::GetViewPortSize() const {
  if (view_port_size_.has_value()) {
    return *view_port_size_;
  } else {
    imp::float2 viewport_size =
        GetView().GetDevice().PixelsToPhysicalPixels(GetView().GetSize());
    return {static_cast<uint32_t>(viewport_size.x),
            static_cast<uint32_t>(viewport_size.y)};
  }
}

filament::View* SurfaceRenderer::GetFilamentView() const {
  return filament_view_;
}

std::optional<int> SurfaceRenderer::GetDisplayId() const { return display_id_; }

void SurfaceRenderer::Render() {
  filament::Renderer* renderer = GetView().GetHost()->GetRenderer();
  // Find the scene for the group this pass renders.
  filament::Scene* scene = GetView().GetGroupsManager().GetScene(group_);

  // If there is no scene, return early without rendering.
  // This is not an error, it means that there are no nodes in the
  // group for this pass, which could be valid (i.e. outlines
  // currently disabled).
  if (!scene) {
    return;
  }

  filament_view_->setScene(scene);

  filament::Camera* camera = camera_->GetCamera();
  double fov = camera->getFieldOfViewInDegrees(filament::Camera::Fov::VERTICAL);
  double near = camera->getNear();
  double far = camera->getCullingFar();
  double original_aspect_ratio =
      GetView().GetSize().x * 1.0 / GetView().GetSize().y;

  if (view_port_size_.has_value()) {
    // If a custom view port size is set, change the camera's aspect ratio to
    // refect the viewport's aspect ratio.
    camera->setProjection(fov,
                          (*view_port_size_).x * 1.0 / (*view_port_size_).y,
                          near, far, filament::Camera::Fov::VERTICAL);
  }

  renderer->beginFrame(swap_chain_, 0);
  renderer->render(filament_view_);
  renderer->endFrame();

  if (view_port_size_.has_value()) {
    // Set the camera's aspect ratio back to original value.
    camera->setProjection(fov, original_aspect_ratio, near, far,
                          filament::Camera::Fov::VERTICAL);
  }
}

ComponentHandle<CameraComponent> SurfaceRenderer::FindCameraWithNodeName(
    absl::string_view camera_node_name) {
  ComponentHandle<CameraComponent> camera;

  // Try to find a CameraComponent that's on a node with camera_node_name
  std::vector<NodeHandle> camera_node_candidates =
      GetView().GetPathManager().FindAll(camera_node_name);

  // Check if any of the node has a CameraComponent. The first one found
  // will be assigned to the filament::View.
  for (NodeHandle candidate : camera_node_candidates) {
    camera = candidate->GetComponent<CameraComponent>();
    if (camera) {
      break;
    }
  }

  return camera;
}

}  // namespace imp
