/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_QUAD_LAYER_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_QUAD_LAYER_RENDERER_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "absl/status/status.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "filament/filament/include/filament/View.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/groups_manager.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_native_window.h"
#include "core/view/platforms/xr_android/xr_session_host.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"

namespace imp {

// XrQuadLayerRenderer renders with a specified Camera to a dedicated OpenXR
// swapchain that is submitted to the OpenXR runtime in xrEndFrame. The quad
// layer's pose and size are automatically synchronized with the world transform
// of the Node this component is attached to.
//
// This class mirrors the API of XrCompositionLayerQuad. See
// https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrCompositionLayerQuad.html
class XrQuadLayerRenderer : public Component {
 public:
  struct XrQuadLayerRendererOptions {
    // The size, in pixels, of the quad layer. Will use the display size if
    // unset.
    std::optional<int2> quad_size;

    // The sample count for the quad layer.
    int sample_count = 1;

    // A weight which will be passed into XrSessionHost::AddCompositionLayer.
    // The main projection layer is drawn with a weight of 0. Layers with lower
    // weights are drawn in front of layers with higher weights.
    int weight;

    // The camera to use for rendering the quad layer. If unset, defaults to the
    // active camera in the view.
    ComponentHandle<CameraComponent> camera;

    // The group to render to the quad layer. Defaults to the main group.
    std::string group = std::string(GroupsManager::kMainGroupName);

    // Swapchain flags passed to Filament when creating the swapchain.
    uint64_t swap_chain_flags = filament::SwapChain::CONFIG_TRANSPARENT;

    // Layer flags passed to OpenXR when creating the XrCompositionLayerQuad.
    // See
    // https://registry.khronos.org/OpenXR/specs/0.90/man/html/XrCompositionLayerFlags.html
    XrCompositionLayerFlags layer_flags = 0;

    // The eye visibility, passed to OpenXR when creating the
    // XrCompositionLayerQuad. See
    // https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrEyeVisibility.html
    XrEyeVisibility eye_visibility = XR_EYE_VISIBILITY_BOTH;

    // The override mode for the dedicated Filament view.
    render_settings::OverrideMode override_mode =
        render_settings::OverrideMode::OVERRIDE_MODE_OVERRIDE_CURRENT;

    // Custom render settings for the dedicated Filament view.
    render_settings::ViewRenderSettings view_render_settings;
  };

  // Creates a swapchain through OpenXR, and assigns it to the given view.
  Future<absl::Status> Setup(const XrQuadLayerRendererOptions& options);

  virtual ~XrQuadLayerRenderer() = default;

  // Destroys the swapchain and filament view.
  void Cleanup();

  void OnActiveStatusChanged(bool is_active);

 private:
  // Updates the quad layer's transform to match that of the Node.
  void UpdateQuadLayerTransform();

  // Renders to the quad layer.
  void Render();

  // Initializes the quad layer after the swapchain is created.
  void InitializeQuadLayer(const XrSwapchain& swapchain,
                           const XrQuadLayerRendererOptions& options);

  XrSessionHost* GetXrSessionHost();

  void AddCompositionLayer();
  void RemoveCompositionLayer();

  bool is_composition_layer_added_ = false;

  filament::View* filament_view_;
  filament::SwapChain* swap_chain_;
  std::unique_ptr<XrNativeWindow> native_window_;
  ComponentHandle<CameraComponent> camera_;
  std::string group_;
  int2 quad_size_;
  XrCompositionLayerQuad quad_layer_{};
  int weight_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_QUAD_LAYER_RENDERER_H_
