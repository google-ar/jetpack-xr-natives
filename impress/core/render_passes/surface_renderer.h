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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_SWAP_CHAIN_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_SWAP_CHAIN_RENDERER_H_

#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/render_passes/proto/surface_renderer_scripting.proto.imp.h"
#include "imp.h"

namespace imp {

using surface_renderer::SurfaceRendererSettings;

// Renders the given group into the given surface in a separate pass.
// Any nodes in the given group will show up in the scene that is
// rendered to the surface. This render pass occurs after the main render pass.
// Note: this can be created from Java through view.setupSwapChainRenderer().
class SurfaceRenderer : public Component {
 public:
  virtual ~SurfaceRenderer() = default;

  struct ViewPortSizeChangedEvent : public Event {
    explicit ViewPortSizeChangedEvent(imp::uint2 new_view_port_size)
        : view_port_size(new_view_port_size) {}

    imp::uint2 view_port_size;
  };

  // Create a SurfaceRenderer targeting a native_window (Surface pointer) and
  // group. Then search for a CameraComponent that's attched on a
  // node with camera_name. If it fails to find such CameraComponent,
  // absl::NotFoundError will be returned and the component will not be added.
  // If std::nullopt is passed in for group, the main
  // group will be used. If std::nullopt is passed in for camera_name, the main
  // camera will be used.
  absl::Status Setup(void* native_window,
                     std::optional<absl::string_view> group,
                     std::optional<absl::string_view> camera_name);
  // Create a SurfaceRenderer targeting a native_window (Surface pointer) and
  // group. If std::nullopt is passed in for group, the
  // main group will be used. The main camera will be used.
  absl::Status Setup(void* native_window,
                     std::optional<absl::string_view> group);
  // Create a SurfaceRenderer targeting a native_window (Surface pointer),
  // group and a camera. If std::nullopt is passed in for
  // group, the main group will be used.
  absl::Status Setup(void* native_window,
                     std::optional<absl::string_view> group,
                     ComponentHandle<CameraComponent> camera);
  // Create a SurfaceRenderer targeting a native_window (Surface pointer) and
  // SurfaceRendererSettings. group and camera can be specified
  // inside SurfaceRendererSettings.
  absl::Status Setup(void* native_window,
                     const SurfaceRendererSettings& settings);

  void Update(const imp::FrameTime& frame_time);

  void Cleanup();

  // Specify the camera to use by camera node name. This function will search
  // for a CameraComponent that's attched on a node with camera_name. If it
  // fails to find such CameraComponent, absl::NotFoundError will be returned.
  absl::Status SetCamera(absl::string_view camera_node_name);

  // Specify the camera to use by directly passing in a camera component handle.
  void SetCamera(ComponentHandle<CameraComponent> camera);

  // Specify the group that's going to be rendered. Use
  // Node::kMainGroupName to specify the main group.
  void SetGroup(absl::string_view group);

  // Specify the view port size.
  void SetViewPortSize(uint2 view_port_size);

  // Returns the CameraComponent that is used for rendering.
  ComponentHandle<CameraComponent> GetCamera() const;

  // Returns the group that is used for rendering.
  absl::string_view GetGroup() const;

  // Returns the view port size.
  uint2 GetViewPortSize() const;

  // Returns the Filament view that is used by this SurfaceRenderer.
  filament::View* GetFilamentView() const;

  // Returns the display id if specified in SurfaceRendererSettings.
  std::optional<int> GetDisplayId() const;

 protected:
  // Renders the target group to the target surface.
  virtual void Render();

 private:
  ComponentHandle<CameraComponent> FindCameraWithNodeName(
      absl::string_view camera_node_name);

  ComponentHandle<CameraComponent> camera_;
  std::string group_;
  filament::SwapChain* swap_chain_;
  filament::View* filament_view_;
  std::optional<imp::uint2> view_port_size_;
  std::optional<int> display_id_;

  // TODO: Remove render_this_frame_ and find a better solution.
  bool render_this_frame_ = false;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_SWAP_CHAIN_RENDERER_H_
