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
#include <utility>

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
#include "core/camera/camera_state.proto.imp.h"
#include "core/config.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_native_window.h"
#include "core/view/platforms/xr_android/xr_session_host.h"
#include "core/view/utils/frame_time.h"
#include "core/view/utils/render_setting_utils.h"
#include "core/xr/openxr_events.h"

namespace imp {

Future<absl::Status> XrQuadLayerRenderer::Setup(
    const XrQuadLayerRendererOptions& options) {
  if (!GetView().GetHost()->IsInXr()) {
    return absl::InternalError("XrQuadLayerRenderer may only be used in XR.");
  }

  options_ = options;

  camera_ = options_.camera ? options_.camera
                            : GetView().GetCameraManager().GetCamera();

  filament::Engine& engine = *BaseView::GetSharedEngine();
  empty_scene_ = engine.createScene();
  filament_view_ = GetView().CreateFilamentView();
  filament_view_->setCamera(camera_->GetCamera());

  ConfigureViewRenderSettingsWithOverrides(
      options.override_mode, filament_view_, GetView().GetHost()->GetView(),
      &options.view_render_settings, &engine);

  imp::Dispatcher& dispatcher = GetView().GetDispatcher();
  dispatcher.Connect(
      [this](const XrCompositionLayerRenderEvent& event) { Render(); }, this);

  return Resize(options.quad_size.value_or(GetView().GetSize())).Then([this]() {
    SetRenderBehavior(options_.render_behavior);
    return absl::OkStatus();
  });
}

void XrQuadLayerRenderer::Update(const FrameTime& frame_time) {
  UpdateQuadLayerTransform();

  bool should_add_to_host =
      render_behavior_ != RenderBehavior::kDisabled && is_swapchain_ready_;
  if (should_add_to_host && !is_added_to_host_) {
    GetXrSessionHost()->AddCompositionLayer(
        reinterpret_cast<XrCompositionLayerBaseHeader*>(&quad_layer_),
        options_.weight, true);
    is_added_to_host_ = true;
  } else if (!should_add_to_host && is_added_to_host_) {
    GetXrSessionHost()->RemoveCompositionLayer(
        reinterpret_cast<XrCompositionLayerBaseHeader*>(&quad_layer_));
    is_added_to_host_ = false;
  }
}

void XrQuadLayerRenderer::InitializeQuadLayer(
    const XrSwapchain& swapchain, int2 size,
    const XrQuadLayerRendererOptions& options) {
  quad_layer_.type = XR_TYPE_COMPOSITION_LAYER_QUAD;
  quad_layer_.next = nullptr;
  quad_layer_.space = GetXrSessionHost()->GetXrSpace();
  quad_layer_.eyeVisibility = options.eye_visibility;
  quad_layer_.layerFlags = options.layer_flags;
  quad_layer_.subImage.swapchain = swapchain;
  quad_layer_.subImage.imageRect = {
      {0, 0}, {static_cast<int32_t>(size.x), static_cast<int32_t>(size.y)}};
  quad_layer_.subImage.imageArrayIndex = 0;

  is_swapchain_ready_ = true;
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
  if (is_added_to_host_) {
    GetXrSessionHost()->RemoveCompositionLayer(
        reinterpret_cast<XrCompositionLayerBaseHeader*>(&quad_layer_));
    is_added_to_host_ = false;
  }
  // Cleanup filament-owned memory.
  if (swap_chain_) {
    GetView().GetSharedEngine()->destroy(swap_chain_);
  }
  if (filament_view_) {
    GetView().DestroyFilamentView(filament_view_);
  }
  if (empty_scene_) {
    GetView().GetSharedEngine()->destroy(empty_scene_);
  }
}

void XrQuadLayerRenderer::Render() {
  if (!is_added_to_host_) {
    return;
  }

  // Force at least one render to ensure that the OpenXR runtime initializes
  // this swapchain, and that we can submit it without rendering in subsequent
  // frames.
  if (render_behavior_ != RenderBehavior::kRenderAndSubmit &&
      has_swapchain_rendered_) {
    GetXrSessionHost()->MarkCompositionLayerRenderSkipped();
    return;
  }

  filament::Scene* scene =
      GetView().GetGroupsManager().GetScene(options_.group);
  if (!scene) {
    scene = empty_scene_;
  }

  filament_view_->setScene(scene);
  filament::Camera* camera = camera_->GetCamera();
  filament::Renderer* renderer = GetView().GetHost()->GetRenderer();

#if IMP_MATERIAL_API(VULKAN)
  // The vulkan backend will not call EndFrame() if beginFrame() fails, so we
  // need to mark the composition layer as skipped here.
  if (!renderer->beginFrame(swap_chain_, 0)) {
    GetXrSessionHost()->MarkCompositionLayerRenderSkipped();
  }
#else
  renderer->beginFrame(swap_chain_, 0);
#endif

  filament::Renderer::ClearOptions original_clear_options =
      renderer->getClearOptions();
  renderer->setClearOptions(options_.clear_options);

  const double near = camera->getNear();
  const double far = camera->getCullingFar();
  const double aspect_ratio = quad_layer_.subImage.imageRect.extent.width *
                              1.0 /
                              quad_layer_.subImage.imageRect.extent.height;
  const double original_aspect_ratio =
      GetView().GetSize().x * 1.0 / GetView().GetSize().y;

  bool is_orthographic =
      camera_->GetProjectionType() == CameraState::ORTHOGRAPHIC;

  // We need to update the camera's projection matrix to match the quad layer's
  // aspect ratio. For an orthographic camera, we can just pass in the
  // dimensions. For a perspective camera, we need to adjust the FOV.
  if (is_orthographic) {
    double height = camera_->GetOrthographicScale();
    double width = height * aspect_ratio;
    double half_width = width * 0.5;
    double half_height = height * 0.5;
    camera->setProjection(filament::Camera::Projection::ORTHO, -half_width,
                          half_width, -half_height, half_height, near, far);

    renderer->render(filament_view_);

    double original_width = height * original_aspect_ratio;
    double half_orig_width = original_width * 0.5;
    camera->setProjection(filament::Camera::Projection::ORTHO, -half_orig_width,
                          half_orig_width, -half_height, half_height, near,
                          far);
  } else {
    const double fov =
        camera->getFieldOfViewInDegrees(filament::Camera::Fov::VERTICAL);
    camera->setProjection(fov, aspect_ratio, near, far,
                          filament::Camera::Fov::VERTICAL);
    renderer->render(filament_view_);
    camera->setProjection(fov, original_aspect_ratio, near, far,
                          filament::Camera::Fov::VERTICAL);
  }

  renderer->setClearOptions(original_clear_options);
  renderer->endFrame();

  has_swapchain_rendered_ = true;
}

Future<absl::Status> XrQuadLayerRenderer::Resize(int2 new_size) {
  pending_resize_future_ = pending_resize_future_.Then(
      [this, new_size]() { return ResizeImpl(new_size); });

  return pending_resize_future_;
}

Future<absl::Status> XrQuadLayerRenderer::ResizeImpl(int2 new_size) {
  imp::Future<XrSwapchain> swapchain_future;
  auto new_native_window = std::make_unique<XrNativeWindow>(XrNativeWindow{
      .host = GetXrSessionHost(),
      .quad_layer_data =
          XrNativeWindow::QuadLayerData{
              .size = new_size,
              .sample_count = options_.sample_count,
              .swapchain = swapchain_future,
          },
  });

  filament::SwapChain* old_swap_chain = swap_chain_;
  filament::Engine& engine = *BaseView::GetSharedEngine();
  // Trigger asynchronous swapchain creation.
  filament::SwapChain* new_swap_chain = engine.createSwapChain(
      new_native_window.get(), options_.swap_chain_flags);

  // std::move and capture the native window so it's not destroyed until the new
  // swapchain is created and Filament is done with it.
  return swapchain_future.Then(
      [this, new_swap_chain, old_swap_chain,
       new_native_window = std::move(new_native_window),
       new_size](XrSwapchain handle) {
        swap_chain_ = new_swap_chain;
        filament_view_->setViewport({0, 0, static_cast<uint32_t>(new_size.x),
                                     static_cast<uint32_t>(new_size.y)});
        InitializeQuadLayer(handle, new_size, options_);
        has_swapchain_rendered_ = false;
        UpdateQuadLayerTransform();

        if (old_swap_chain) {
          GetXrSessionHost()->DeferSwapChainDestruction(old_swap_chain);
        }
      });
}

XrSessionHost* XrQuadLayerRenderer::GetXrSessionHost() {
  // This is safe since we check that the host is in XR in Setup().
  return static_cast<XrSessionHost*>(GetView().GetHost());
}

void XrQuadLayerRenderer::SetRenderBehavior(RenderBehavior render_behavior) {
  render_behavior_ = render_behavior;
}

XrQuadLayerRenderer::RenderBehavior XrQuadLayerRenderer::GetRenderBehavior()
    const {
  return render_behavior_;
}

}  // namespace imp
