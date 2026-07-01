// Copyright 2026 Google LLC
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
#include "core/view/platforms/xr_android/xr_quad_layer_renderer.h"

#include <cstdint>
#include <memory>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Scene.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/Viewport.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/camera/camera_manager.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_events.proto.imp.h"
#include "core/view/platforms/xr_android/xr_native_window.h"
#include "core/view/platforms/xr_android/xr_session_host.h"
#include "core/view/utils/render_setting_utils.h"

namespace imp {

Future<absl::Status> XrQuadLayerRenderer::Setup(
    const XrQuadLayerRendererOptions& options) {
  if (!GetView().GetHost()->IsInXr()) {
    return absl::InternalError("XrQuadLayerRenderer may only be used in XR.");
  }

  weight_ = options.weight;
  group_ = options.group;
  camera_ = options.camera ? options.camera
                           : GetView().GetCameraManager().GetCamera();
  quad_size_ = options.quad_size.value_or(GetView().GetSize());

  filament::Engine& engine = *BaseView::GetSharedEngine();
  filament_view_ = engine.createView();
  filament_view_->setCamera(camera_->GetCamera());
  filament_view_->setViewport({0, 0, static_cast<uint32_t>(quad_size_.x),
                               static_cast<uint32_t>(quad_size_.y)});

  ConfigureViewRenderSettingsWithOverrides(
      options.override_mode, filament_view_, GetView().GetHost()->GetView(),
      &options.view_render_settings, &engine);

  // Because swapchain creation happens asynchronously, use a future to
  // asynchronously handle the XrSwapchain.
  imp::Future<XrSwapchain> swapchain_future;
  native_window_ = std::unique_ptr<XrNativeWindow>(new XrNativeWindow{
      .host = GetXrSessionHost(),
      .quad_layer_data =
          XrNativeWindow::QuadLayerData{
              .size = quad_size_,
              .sample_count = options.sample_count,
              .swapchain = swapchain_future,
          },
  });
  swap_chain_ =
      engine.createSwapChain(native_window_.get(), options.swap_chain_flags);

  // Initialize the quad layer on the foreground thread once the swapchain is
  // created.
  return swapchain_future.Then([this, options](XrSwapchain handle) {
    return InitializeQuadLayer(handle, options);
  });
}

void XrQuadLayerRenderer::InitializeQuadLayer(
    const XrSwapchain& swapchain, const XrQuadLayerRendererOptions& options) {
  quad_layer_.type = XR_TYPE_COMPOSITION_LAYER_QUAD;
  quad_layer_.next = nullptr;
  quad_layer_.space = GetXrSessionHost()->GetXrSpace();
  quad_layer_.eyeVisibility = options.eye_visibility;
  quad_layer_.layerFlags = options.layer_flags;
  quad_layer_.subImage.swapchain = swapchain;
  quad_layer_.subImage.imageRect = {
      {0, 0},
      {static_cast<int32_t>(quad_size_.x), static_cast<int32_t>(quad_size_.y)}};
  quad_layer_.subImage.imageArrayIndex = 0;

  // Match the transform of the node.
  UpdateQuadLayerTransform();

  // Register the quad layer with the XrSessionHost.
  AddCompositionLayer();

  imp::Dispatcher& dispatcher = GetView().GetDispatcher();
  dispatcher.Connect(
      [this](const XrPreAdvanceFrameEvent& event) {
        if (IsActive() && !event.frame_skipped_reason.has_value()) {
          // This is called once per frame, and happens before the main
          // projection layer is rendered.
          // The order of operations is:
          // 1. Quad layer triggers xrBeginFrame and renders.
          // 2. Main projection layer renders.
          // 3. Main projection layer triggers xrEndFrame.
          Render();
        }
      },
      this);
}

void XrQuadLayerRenderer::UpdateQuadLayerTransform() {
  NodeHandle node = GetNode();
  const float3 world_transform = node->GetWorldPosition();
  const quatf world_rotation = node->GetWorldRotation();
  const float3 world_scale = node->GetWorldScale();
  quad_layer_.pose.position = {world_transform.x, world_transform.y,
                               world_transform.z};
  // XrPosef::orientation (XrQuaternionf) maps to Node rotation (quatf).
  quad_layer_.pose.orientation = {world_rotation.x, world_rotation.y,
                                  world_rotation.z, world_rotation.w};
  // XrCompositionLayerQuad::size (XrExtent2Df) maps to Node scale (float3).
  quad_layer_.size = {world_scale.x, world_scale.y};
}

void XrQuadLayerRenderer::Cleanup() {
  RemoveCompositionLayer();
  // Cleanup filament-owned memory.
  if (swap_chain_) {
    GetView().GetSharedEngine()->destroy(swap_chain_);
  }
  if (filament_view_) {
    GetView().GetSharedEngine()->destroy(filament_view_);
  }
}

void XrQuadLayerRenderer::Render() {
  filament::Scene* scene = GetView().GetGroupsManager().GetScene(group_);
  if (!scene) {
    // If there is no scene, there is nothing to render.
    return;
  }
  UpdateQuadLayerTransform();

  filament_view_->setScene(scene);
  filament::Camera* camera = camera_->GetCamera();
  filament::Renderer* renderer = GetView().GetHost()->GetRenderer();
  renderer->beginFrame(swap_chain_, 0);

  // Adjust the camera's projection matrix to match the quad layer.
  const double fov =
      camera->getFieldOfViewInDegrees(filament::Camera::Fov::VERTICAL);
  const double near = camera->getNear();
  const double far = camera->getCullingFar();
  const double original_aspect_ratio =
      GetView().GetSize().x * 1.0 / GetView().GetSize().y;
  const double aspect_ratio = quad_layer_.subImage.imageRect.extent.width *
                              1.0 /
                              quad_layer_.subImage.imageRect.extent.height;

  // Set the projection matrix, render, and restore the original.
  camera->setProjection(fov, aspect_ratio, near, far,
                        filament::Camera::Fov::VERTICAL);
  renderer->render(filament_view_);
  camera->setProjection(fov, original_aspect_ratio, near, far,
                        filament::Camera::Fov::VERTICAL);
  renderer->endFrame();
}

void XrQuadLayerRenderer::OnActiveStatusChanged(bool is_active) {
  if (is_active) {
    AddCompositionLayer();
  } else {
    RemoveCompositionLayer();
  }
}

void XrQuadLayerRenderer::AddCompositionLayer() {
  if (is_composition_layer_added_) {
    return;
  }
  GetXrSessionHost()->AddCompositionLayer(
      reinterpret_cast<XrCompositionLayerBaseHeader*>(&quad_layer_), weight_);
  is_composition_layer_added_ = true;
}

void XrQuadLayerRenderer::RemoveCompositionLayer() {
  if (!is_composition_layer_added_) {
    return;
  }
  GetXrSessionHost()->RemoveCompositionLayer(
      reinterpret_cast<XrCompositionLayerBaseHeader*>(&quad_layer_));
  is_composition_layer_added_ = false;
}

XrSessionHost* XrQuadLayerRenderer::GetXrSessionHost() {
  // This is safe since we check that the host is in XR in Setup().
  return static_cast<XrSessionHost*>(GetView().GetHost());
}

}  // namespace imp
