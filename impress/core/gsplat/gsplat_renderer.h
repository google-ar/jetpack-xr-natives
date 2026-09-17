/*
 * Copyright 2025 Google LLC
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
#ifndef THIRD_PARTY_IMPRESS_CORE_GSPLAT_GSPLAT_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_GSPLAT_GSPLAT_RENDERER_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/time/time.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/camera/camera_manager.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/gsplat/gsplat.pb.h"
#include "core/gsplat/gsplat_asset.h"
#include "core/gsplat/gsplat_renderer_state.proto.imp.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/render/mesh_renderer.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/gsplat/precompute_texture_pipeline.h"
#include "core/view/utils/frame_time.h"

#if IMP_RUNTIME(DEV)
#include "core/common/registry.h"
#include "core/editor/editor.h"
#endif

namespace filament {
class View;
}

#if IMP_RUNTIME(DEV)
#include "core/editor/file_type.h"
#endif

namespace imp {

// A component that implements Gaussian Splatting rendering given a GSplatAsset
// containing the scene to render.
// Doc: (broken link)
class GSplatRenderer : public Component {
 public:
  Future<absl::Status> Setup();

  // Creates Gsplat renderer from an existing GSplatAsset.
  // For basic gsplat use cases.  The default render and precompute materials
  // will be used. An internal texture rendering pipeline will be set up to do
  // precomputation.  Not split engine compatible, because the default render
  // material is not compatible.
  // TODO: (broken link) - Replace default use of GsplatMaterialSerializer with
  // direct use of render and precompute materials.
  Future<absl::Status> Setup(AssetPtr<GSplatAsset> gsplat_asset);

  // Creates the Gsplat renderer with the provided combined render and
  // precompute material.  Split Engine compatible if the material used is split
  // engine compatible.  GsplatMaterialSerializer is recommended for Split
  // Engine use. Note that GsplatMaterialSerializer is more than just rendering
  // instructions.  It is responsible for IPC and will trigger the creation of a
  // graph of other components and resources to manage precompute and rendering.
  // TODO: (broken link) - Add optional combined material without precompute pass
  // TODO: (broken link) - Accept nullptr material to use default material.
  Future<absl::Status> Setup(
      AssetPtr<GSplatAsset> gsplat_asset,
      OwnedMaterialPtr combined_render_and_precompute_material);

  // Creates the Gsplat renderer with client provided render and precompute
  // material separates.  Use for highly customized use cases.
  // Not compatible with Split Engine.
  // TODO: (broken link) - Accept nullptr materials to use default materials.
  Future<absl::Status> Setup(AssetPtr<GSplatAsset> gsplat_asset,
                             OwnedMaterialPtr render_material,
                             OwnedMaterialPtr precompute_material);

  // Creates Gsplat renderer using GSplatRendererState.
  // TODO: (broken link) - Replace default use of GsplatMaterialSerializer with
  // direct use of render and precompute materials.
  Future<absl::Status> SetupWithState();

  void Update(const FrameTime& frame_time);
  void Cleanup();

  BorrowedMaterialPtr GetRenderMaterial(
      imp::SmallSourceLocation loc = imp::SmallSourceLocation()) const;

  // Returns the precomputed material override, or null if none is given at
  // Setup().
  BorrowedMaterialPtr GetPrecomputeMaterial(
      imp::SmallSourceLocation loc = imp::SmallSourceLocation()) const;

  // Returns the opacity scale of the gaussians.
  float GetOpacityScale() const { return *state_.opacity_scale; }

  // Sets the opacity scale of the gaussians.
  void SetOpacityScale(float opacity_scale) {
    state_.opacity_scale = opacity_scale;
  }

  ComponentHandle<CameraComponent> GetCamera() const {
    ComponentHandle<CameraComponent> override_camera = state_.override_camera;
#if IMP_RUNTIME(DEV)
    ComponentHandle<CameraComponent> camera = override_camera
                                                  ? override_camera
                                                  : GetView()
                                                        .GetRegistry()
                                                        .Get<editor::Editor>()
                                                        ->get()
                                                        .GetActiveCamera();
#else
    ComponentHandle<CameraComponent> camera =
        override_camera ? override_camera
                        : GetView().GetCameraManager().GetCamera();
#endif
    return camera;
  }

  // Returns the camera used for rendering and depth sorting if set, otherwise
  // nullptr.
  ComponentHandle<CameraComponent> GetOverrideCamera() const {
    return state_.override_camera;
  }

  // Sets the camera used for rendering and depth sorting. If null, the main
  // view's camera will be used.
  void SetOverrideCamera(ComponentHandle<CameraComponent> camera) {
    state_.override_camera = camera;
    if (precompute_texture_pipeline_) {
      (void)precompute_texture_pipeline_->SetCamera(camera);
    }
  }

  // Returns the active view used for viewport size dependent math.
  filament::View* GetFilamentView() const {
    return override_view_ ? override_view_ : GetView().GetHost()->GetView();
  }

  // Returns the override view used for viewport size dependent math if set,
  // otherwise nullptr.
  filament::View* GetOverrideFilamentView() const { return override_view_; }

  // Sets the view used for viewport size dependent math. This is used for
  // scenarios where you are rendering the gsplat to a render target that isn't
  // the main view.
  // This is only used in non-Split Engine use cases, since the overriding view
  // resolution cannot be passed to Split Engine system through the public
  // Gsplat API.
  void SetOverrideFilamentView(filament::View* view) { override_view_ = view; }

  // Returns the render priority of the gsplat.
  uint8_t GetPriority() const { return mesh_renderer_->GetPriority(); }

  // Sets the render priority of the gsplat.
  void SetPriority(uint8_t priority) { mesh_renderer_->SetPriority(priority); }

  // Sets the global blend order of the gsplat.
  void SetBlendOrder(uint16_t blend_order) {
    mesh_renderer_->SetBlendOrder(blend_order,
                                  imp::MeshRenderer::BlendOrderMode::kGlobal);
  }

  // Specifies PrecomputeTexturePipeline component won't be removed until after
  // this component. This is to ensure that we have control over destruction of
  // the component and it's dependencies in Cleanup() to ensure that the texture
  // owned by the precompute texture pipeline (via the texture registry) is not
  // destroyed before any material used on this renderer that is borrowing the
  // texture is destroyed.
  using CleanupDependents =
      imp::CleanupIds<imp::split_engine::PrecomputeTexturePipeline>;

 private:
  // Sort the positions of the splats back-to-front for rendering if needed. The
  // sort operation may be asynchronous and the state of the current sort
  // operation is stored in sort_future_.
  // TODO: (broken link) - Investigate switching to front-to-back
  // sorting/rendering with custom blending mode into separate texture using
  // oneMinusDstAlpha, like 3DGS paper.
  void MaybeSortGaussians();

  // Asynchronously sorts the splats back-to-front.
  void SortGaussians(ComponentHandle<CameraComponent> camera);

  // Whether sorting of the gaussians is necessary because of camera movement.
  bool ShouldSortGaussians(ComponentHandle<CameraComponent> camera,
                           const FrameTime& frame_time);

  // Generates the sorted_indices_ vector and runs a first pass of sorting.
  Future<absl::Status> InitializeSortedIndices();

  // Update material parameters at runtime from the state proto.
  absl::Status UpdateMaterialParameters();

  // Initializes the component once the gaussian splatting material and gsplat
  // dataset are fully loaded.
  Future<absl::Status> LoadFromResources();

  // Whether the bounding box of the gsplat is completely outside the camera FoV
  bool IsGsplatAabbOutsideFov(
      ComponentHandle<imp::CameraComponent> camera) const;

  // The gsplat asset containing the data to render.
  AssetPtr<GSplatAsset> gsplat_asset_;
  // The indices into the positions_ vector we use to render the gaussians
  // back-to-front. We cache this as an optimization for the sort - since camera
  // movement between sort operations is usually relatively small, the array is
  // mostly sorted from the previous sort operation, and starting with a mostly
  // sorted array improve sort performance. We use a shared_ptr to avoid copying
  // the data to the background thread when sorting is done asynchronously,
  // while also ensuring the lifetime of the data is managed correctly (for
  // example, if gsplat renderer is destroyed before a sort operation is
  // completed). This data should only be accessed/modified within sort_future_.
  std::shared_ptr<std::vector<uint32_t>> sorted_indices_ =
      std::make_shared<std::vector<uint32_t>>();
  // The underlying mesh renderer used to render the gaussian splat scene.
  ComponentHandle<MeshRenderer> mesh_renderer_;
  // Propagates changes to the material used to render the gaussian splat scene.
  // Ignored if combined_render_and_precompute_material_ is set.
  OwnedMaterialPtr render_material_;
  // Combined material used for both rendering and precomputation (e.g. Split
  // Engine).
  // combined_render_and_precompute_material_ has the highest priority.
  // Render_material_ and precompute_material_override_ are only used if
  // combined_render_and_precompute_material_ is not set.
  OwnedMaterialPtr combined_render_and_precompute_material_;
  // The precompute texture pipeline used when a precompute material override is
  // provided.
  ComponentHandle<split_engine::PrecomputeTexturePipeline>
      precompute_texture_pipeline_;
  // An optional override precompute material to use instead of the builtin
  // material managed one.
  // Ignored if combined_render_and_precompute_material_ is set.
  BorrowedMaterialPtr precompute_material_override_;
  // Cache of the last world space camera forward vector we've sorted at.
  double3 last_compute_depth_forward_ = kZero3;
  // Cache of the last world space camera position vector we've sorted at.
  double3 last_compute_depth_position_ = kZero3;
  // Last time in seconds we sorted the splats.
  absl::Time last_sort_time_ = absl::InfinitePast();
  // The last sort mode we used.
  GSplatRendererState::SortMode last_sort_mode_ = GSplatRendererState::HYBRID;
  // Future containing the state of the latest sort operation.
  Future<absl::Status> sort_future_ = Future<absl::Status>(absl::OkStatus());
  // sorted_indices_texture_ must be destroyed after the material is destroyed.
  // When sorted_indices_texture_ is replaced, care must be taken to ensure it
  // is not destroyed until after the texture has been uploaded.
  OwnedTexturePtr sorted_indices_texture_;
  // The view used for viewport size dependent math. If null, the main view will
  // be used.
  filament::View* override_view_ = nullptr;
  // Node created by this component to host the precompute texture pipeline.
  // Null, if this class didn't create the node. This node will be destroyed in
  // Cleanup().
  NodeHandle precompute_node_;
  GSplatRendererState state_;

 public:
  using IsfInfo = IsfInfo<&GSplatRenderer::state_>;
  static constexpr bool kRunInEditMode = true;

  // Create a friend class we can use to test internal (private) methods
  // (broken link)
  friend class GsplatRendererTestPeer;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GSPLAT_GSPLAT_RENDERER_H_
