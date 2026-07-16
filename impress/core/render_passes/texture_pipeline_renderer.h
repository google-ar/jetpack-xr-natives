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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_TEXTURE_PIPELINE_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_TEXTURE_PIPELINE_RENDERER_H_

#include <cstddef>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/optional.h"
#if defined(__ANDROID__)
#include "filament/filament/include/filament/Fence.h"
#endif  // defined(__ANDROID__)
#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/View.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/isf_info.h"
#include "core/render/mesh_renderer.h"
#include "core/render/primitive_shape_renderer.h"
#include "core/render/texture.h"
#include "core/render/texture_registry.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/render/render_component.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"

namespace imp {

// Forward declaration of peer class to support testing.
class TexturePipelineRendererPeer;

// Component for rendering a sequence of passes into offscreen textures prior to
// the main scene being rendered.
//
// This can be used to create visual effects that require multiple render passes
// like outlines and mirrors.
//
// Each pass can output color & depth textures that can then be used as inputs
// into subsequent passes or the main scene.
//
// Each pass can specify a group to control which set of nodes get
// rendered in which pass.
class TexturePipelineRenderer : public Component {
  friend class TexturePipelineRendererTest;

 public:
  // PostRenderEvent will be sent out to the owning node of
  // TexturePipelineRenderer in RenderPasses() after the passes are rendered.
  class PostRenderEvent : public Event {
   public:
    PostRenderEvent() = default;
  };

  // PostResizeEvent will be sent out to the owning node of
  // TexturePipelineRenderer after textures are resized during a resize event.
  class PostResizeEvent : public Event {
   public:
    PostResizeEvent() = default;
  };

  // Pre and post render events. Any material overrides made during the
  // PrePassEvent should usually be undone in PostPassEvent. The
  // `TexturePipelineRenderer` material override is treated as a default
  // override, and is applied to the group being rendered before PrePassEvent is
  // sent, and undone after PostPassEvent.
  //
  // NOTE: TexturePipelineRenderer passes are rendered during the main frame
  // loop (after beginFrame is called), when material uniform parameter values
  // are latched. This means any material parameter changes in PrePassEvent will
  // be applied in the next frame.
  //
  // Therefore, a single Material instance can't be reused between passes if
  // they require different parameter values. Instead, use a separate material
  // instance per pass.
  struct PrePassEvent : public Event {
    size_t pass_index = 0;
  };
  struct PostPassEvent : public Event {
    size_t pass_index = 0;
  };

  Future<absl::Status> Setup();

  // Enables or disables the given pass. `pass_index` must be less than the
  // number of passes, otherwise this is a no-op.
  void SetPassEnabled(size_t pass_index, bool enabled);
  // Returns the enabled state of the given pass. `pass_index` must be less than
  // the number of passes, otherwise this returns false.
  bool IsPassEnabled(size_t pass_index) const;

  // Resizes the texture for the given pass. `pass_index` must be less than the
  // number of passes, otherwise this returns an error.
  absl::Status ResizePassTexture(size_t pass_index, imp::uint2 size);

  // Sets the camera for the given pass. If the camera component passed in is
  // not valid, then the main camera will be used. `pass_index` must be less
  // than the number of passes, otherwise this returns an error.
  absl::Status SetPassCamera(size_t pass_index,
                             ComponentHandle<CameraComponent> camera);

  // Returns the Filament view used for the given pass. `pass_index` must be
  // less than the number of passes, otherwise this returns nullptr.
  filament::View* GetFilamentView(size_t pass_index) const;

  void Cleanup();

  // Access the override material for the given pass.
  // Useful for setting parameters on the override material.
  // Returns std::nullopt if there is no override material for the pass.
  std::optional<BorrowedMaterialPtr> GetOverrideMaterial(
      size_t pass_index,
      imp::SmallSourceLocation loc = imp::SmallSourceLocation::Current()) const;

  // Sets a projection quad, or clears the quad if null.
  //
  // When a projection quad is given (specified in the World Space), the
  // TexturePipelineRenderer operates in a special mode to render the scene
  // "through" the given quad with the main camera. This is useful to render a
  // "magic window" like effect by first rendering the scene to a texture, and
  // then the caller can render this texture onto the quad.
  //
  // When projection quad is given, the TexturePipelineRenderer::Pass::camera
  // field will be ignored, since the rendering will be done with the main
  // camera.
  void SetProjectionQuad(
      const std::optional<TexturePipelineRendererProjectionQuad>& quadInWorld);

  // ComponentSystem to handle priority-ordering of TexturePipelineRenderer.
  class System : public ComponentSystem<TexturePipelineRenderer> {
   public:
    explicit System(BaseView* view);

    void BeforeFirstComponentAdded() override;
    void AfterLastComponentRemoved() override;
    // Calls TexturePipelineRenderer::RenderPasses on all enabled
    // TexturePipelineRenderers in priority order.
    void RunTexturePipelines(filament::Renderer* filament_renderer);

    filament::Scene* GetEmptyScene() const { return empty_scene_; }

   private:
    // An empty (but not null) scene to use for passes that don't have any nodes
    // in their group.
    filament::Scene* empty_scene_ = nullptr;

#if defined(__ANDROID__)
    // `first_frame_fence_` and `is_first_frame_rendered_` are used together to
    // track the completion of the first frame rendered by Filament. This is
    // necessary to ensure that the GL context is fully available for off-screen
    // rendering by waiting for the first frame to complete. See (broken link)
    // for more context.
    filament::Fence* first_frame_fence_ = nullptr;
    bool is_first_frame_rendered_ = false;
#endif  // defined(__ANDROID__)
  };

 private:
  // Stores information about each render pass that is used at runtime in
  // conjunction with the data stored in TextureRenderPassesState.
  struct RuntimePass {
    // Passes can be disabled at runtime.
    bool enabled = true;

    // Each pass has a filament View with its own rendering settings. The
    // render target is assigned to this view to control the output. The
    // filament::Scene associated with the group being rendered is
    // also applied to the view.
    filament::View* view = nullptr;

    // This is used to assign the output textures to the pass in filament.
    filament::RenderTarget* render_target = nullptr;

    // Registration for the color texture that the pass outputs, if there is
    // one.
    absl::optional<TextureRegistry::ScopedTextureRegistration>
        color_texture_registration;

    // Registration for the depth texture that the pass outputs, if there is
    // one.
    absl::optional<TextureRegistry::ScopedTextureRegistration>
        depth_texture_registration;

    // Material to use to override all nodes rendered by the pass.
    OwnedMaterialPtr override_material;

    // This will be true if the pass does not render to the entire texture.
    bool rendering_to_subregion = false;

    bool use_main_view_settings = false;
    // If true, the fov and other parameters are automatically set
    // when in OpenXR.  If false, the developer is responsible for setting the
    // projection matrix for the pass camera.  The Default is true.
    bool use_main_view_camera_projection_matrix = true;

    std::optional<render_settings::ViewRenderSettings> render_settings;
  };

  // (Re)initializes the textures and view render targets to the current
  // configured texture size for the given `pass`.
  absl::Status InitializeTextures(
      const TexturePipelineRendererState::Pass& pass,
      RuntimePass& runtime_pass);

  void RenderPasses(filament::Renderer* filament_renderer);

  // Returns true if the texture that our color target writes to has been since
  // deleted and recreated elsewhere in the registry. This indicates that we
  // need to recreate our RenderTarget.
  bool IsRenderTargetStale(const TexturePipelineRendererState::Pass& pass,
                           const RuntimePass& runtime_pass) const;

  // Allow tests to access the state for verification.
  friend class TexturePipelineRendererPeer;
  TexturePipelineRendererState state_;

  std::vector<RuntimePass> runtime_passes_;

  // When the view is resized, we delete and recreate the textures. One problem
  // is that if there are any borrowers of that texture, because it is an
  // OwnedTexturePtr, Impress by design will fatal and crash. To prevent this,
  // we should mark the textures for deletion, and delete them in the
  // PostResizeEvent.
  std::vector<OwnedTexturePtr> textures_marked_for_deletion_;

  // Optional projection quad. See SetProjectionQuad() for more information.
  std::optional<TexturePipelineRendererProjectionQuad> projection_quad_;

 public:
  // PrimitiveShapeRenderer needs to be dependent since it might be using
  // textures created by this component.
  using IsfInfo = IsfInfo<&TexturePipelineRenderer::state_, void,
                          IsfDependents<PrimitiveShapeRenderer>>;
  static constexpr bool kRunInEditMode = true;
  using CleanupDependencies = CleanupIds<RenderComponent, MeshRenderer>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_TEXTURE_PIPELINE_RENDERER_H_
