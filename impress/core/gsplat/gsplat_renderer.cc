// Copyright 2025 Google LLC
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

#include "core/gsplat/gsplat_renderer.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/Options.h"
#include "filament/filament/include/filament/Viewport.h"
#include "filament/libs/math/include/math/mat4.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/camera/camera_component.h"
#include "core/common/owned_ptr.h"
#include "core/common/small_source_location.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/geometry/shapes/box.h"
#include "core/gsplat/encoding.h"
#include "core/gsplat/gsplat_asset.h"
#include "core/gsplat/gsplat_material_params.h"
#include "core/gsplat/gsplat_renderer_state.proto.imp.h"
#include "core/image/borrowed_image_content.h"
#include "core/materials/material.h"
#include "core/math/almost_equal.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_factory.h"
#include "core/model/mesh/vertex_format.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/render/mesh_renderer.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/render/texture_options.h"
#include "core/split_engine/materials/builtin/gsplat/gsplat_material_helpers.h"
#include "core/split_engine/materials/builtin/gsplat/precompute_material_assets.h"
#include "core/split_engine/materials/builtin/gsplat/precompute_texture_pipeline.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/utils/frame_time.h"

namespace imp {
namespace {

// Computes the depths of the gaussians based on the sort mode. Expects camera
// position and camera forward vectors to be in the same coordinate space as the
// positions.
std::vector<float> ComputeDepths(
    const int count, const std::vector<filament::math::half4>& positions,
    GSplatRendererState::SortMode sort_mode, const double3& camera_position,
    const double3& camera_forward) {
  IMP_TRACE();
  std::vector<float> depths;
  depths.resize(count);
  switch (sort_mode) {
    case GSplatRendererState::SortMode::CAMERA_POSITION: {
      for (int x = 0; x < count; ++x) {
        depths[x] = distance2(positions[x].xyz, camera_position);
      }
      break;
    }
    default: {
      for (int x = 0; x < count; ++x) {
        depths[x] = dot(positions[x].xyz - camera_position, camera_forward);
      }
      break;
    }
  }
  return depths;
}

// Sort the indices of the gaussian based on the depths we calculated for each
// gaussian.
void SortIndicesByDepth(const std::vector<float>& depths,
                        std::vector<uint32_t>& sorted_indices) {
  IMP_TRACE();
  // Since sorted_indices represents the data we will store in the
  // data texture, it may contain more elements than depths due
  // to padding, so we can't use sorted_indices_ size directly.
  
  std::sort(sorted_indices.begin(), sorted_indices.begin() + depths.size(),
            [&](uint32_t a, uint32_t b) { return depths[a] > depths[b]; });
}

// Upload the sorted indices data to a texture to use in the material.
// This is done by creating a new texture that will be owned by the material.
// The new texture will only be returned when the upload has been completed and
// the old texture is not referenced.
Future<OwnedTexturePtr> UploadSortedIndices(
    BaseView& view, std::shared_ptr<const std::vector<uint32_t>> sorted_indices,
    Material& material) {
  IMP_TRACE();
  

  const uint2 texture_size = CalculateTextureSize(sorted_indices->size());
  Future<absl::Status> upload_future = Future<absl::Status>();
  image::BorrowedImageContents<uint32_t> sorted_indices_image_contents(
      texture_size.x, texture_size.y, *sorted_indices,
      filament::backend::TextureFormat::R32UI,
      filament::backend::PixelDataFormat::R_INTEGER,
      filament::backend::PixelDataType::UINT,
      [upload_future, sorted_indices]() {
        upload_future.Return(absl::OkStatus());
      });

  OwnedTexturePtr sorted_indices_texture =
      view.GetTextureFactory().CreateTexture(
          sorted_indices_image_contents, TextureGenerationOptions(),
          TextureSamplerOptions{
              .mag_filter = TextureSamplerOptions::MagFilter::NEAREST,
              .min_filter = TextureSamplerOptions::MinFilter::NEAREST},
          "sorted_indices_texture");
  // Since there is heavy churn on this texture, ensure the split engine
  // removal is not suppressed until suppression default is removed: TODO:
  // (broken link)
  sorted_indices_texture->SetSuppressSplitEngineRemoval(false);

  material.SetParameter(kSortedIndicesParameter,
                        sorted_indices_texture.Borrow(),
                        sorted_indices_texture->GetSampler());

  return upload_future.Then(
      [sorted_indices_texture =
           std::move(sorted_indices_texture)]() mutable -> OwnedTexturePtr {
        // Keep the texture alive for the lifetime of the material.
        return std::move(sorted_indices_texture);
      },
      // TODO: (broken link)), kScheduleAlways is necessary currently to ensure
      // the future returns outside of the Filament beginFrame/endFrame, but
      // could be removed once there is a better way to control returning
      // futures from within Filament callbacks.
      FutureThenOptions{.executor_mode = FutureExecutorMode::kScheduleAlways});
}

// Unsets a texture from a material.
// Null textures are not allowed, so a placeholder texture is used instead.
void TryUnsetTexture(BaseView& view, const BorrowedTexturePtr& source_texture,
                     const OwnedMaterialPtr& dest_material,
                     absl::string_view parameter_name) {
  if (source_texture == nullptr || dest_material == nullptr ||
      parameter_name.empty()) {
    // If there is no texture or material, there is nothing to do.
    return;
  }

  // The placeholder's SamplerType and Format must match.
  absl::StatusOr<BorrowedTexturePtr> placeholder =
      view.GetTextureFactory().BorrowMatchingPlaceholderTexture(source_texture);
  if (!placeholder.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to borrow placeholder texture: "
               << placeholder.status();
    return;
  }

  dest_material->TrySetParameter(parameter_name, placeholder.value(),
                                 placeholder.value()->GetSampler());
}

}  // namespace

bool GSplatRenderer::ShouldSortGaussians(
    ComponentHandle<CameraComponent> camera, const FrameTime& frame_time) {
  if (!sort_future_.Ready()) {
    return false;
  }

  // Culling when all splats are outside FoV.
  if (state_.skip_sort_if_outside_fov.Value() &&
      IsGsplatAabbOutsideFov(camera)) {
    return false;
  }

  if (*state_.sort_interval_seconds > 0.0 &&
      frame_time.GetLastTime() <
          last_sort_time_ + absl::Seconds(*state_.sort_interval_seconds)) {
    return false;
  }

  NodeHandle camera_node = camera->GetNode();
  const double3 camera_position = camera_node->GetWorldPositionPrecise();
  const double3 camera_forward = camera_node->GetWorldForward();

  // Don't sort if the camera movement is less than the threshold distance. If
  // the threshold distance is zero (default), then don't sort if the camera
  // position equals the old position (with a tolerance of epsilon).
  const TVec3<double> camera_movement =
      camera_position - last_compute_depth_position_;
  const double threshold = *state_.sort_camera_movement_threshold;
  if (threshold > 0.0 && length2(camera_movement) < threshold * threshold) {
    return false;
  } else if ((last_sort_mode_ == GSplatRendererState::CAMERA_POSITION &&
              AlmostEqual(camera_position, last_compute_depth_position_))) {
    return false;
  }
  if ((last_sort_mode_ == GSplatRendererState::CAMERA_FORWARD &&
       AlmostEqual(camera_forward, last_compute_depth_forward_))) {
    return false;
  }

  return true;
}

void GSplatRenderer::MaybeSortGaussians() {
  IMP_TRACE();
  
  const FrameTime frame_time = GetView().GetFrameTime();

  ComponentHandle<CameraComponent> camera = GetCamera();
  if (!camera) {
    IMP_LOG(imp::ERROR) << "Cannot find a valid camera to sort splats";
    return;
  }

  if (ShouldSortGaussians(camera, frame_time)) {
    SortGaussians(camera);
  }
}

void GSplatRenderer::SortGaussians(ComponentHandle<CameraComponent> camera) {
  IMP_TRACE();
  

  // Can't sort if there's already a sort operation in progress.
  if (!sort_future_.Ready()) {
    return;
  }
  if (!camera) {
    IMP_LOG(imp::ERROR) << "Invalid camera passed to SortGaussians";
    return;
  }
  NodeHandle camera_node = camera->GetNode();
  const double3 camera_position_world = camera_node->GetWorldPositionPrecise();
  const double3 camera_forward_world = camera_node->GetWorldForward();

  // Choose which way to calculate the depth and sort the splats based on camera
  // movement and sort mode priority.
  GSplatRendererState::SortMode sort_mode =
      GSplatRendererState::SortMode::CAMERA_POSITION;
  if (*state_.sort_mode == GSplatRendererState::SortMode::CAMERA_FORWARD ||
      (*state_.sort_mode == GSplatRendererState::SortMode::HYBRID &&
       !AlmostEqual(camera_position_world, last_compute_depth_position_))) {
    sort_mode = GSplatRendererState::CAMERA_FORWARD;
  }

  // Convert camera position and forward vector to gsplat scene relative space.
  const mat4 node_transform = GetNode()->GetWorldTrsPrecise();
  const double3 camera_position_relative =
      (inverse(node_transform) * camera_position_world).xyz;
  const double3 camera_forward_relative =
      (inverse(node_transform) * double4(camera_forward_world, 0.0)).xyz;

  sort_future_ =
      Future<std::shared_ptr<std::vector<uint32_t>>>::Schedule(
          [positions = gsplat_asset_->positions(),
           count = gsplat_asset_->count(), sort_mode,
           sorted_indices = sorted_indices_, camera_position_relative,
           camera_forward_relative]() mutable {
            std::vector<float> depths = ComputeDepths(
                count, *positions, sort_mode, camera_position_relative,
                camera_forward_relative);
            SortIndicesByDepth(depths, *sorted_indices);
            return sorted_indices;
          },
          *state_.sort_in_background ? Executor::Type::kBackground
                                     : Executor::Type::kImmediate)
          .Then(
              [this, sort_mode, camera_position_world, camera_forward_world](
                  std::shared_ptr<std::vector<uint32_t>> result) {
                
                // Since this task executes on the foreground thread, we don't
                // have to lock to update these member variables.
                last_sort_time_ = GetView().GetFrameTime().GetLastTime();
                last_sort_mode_ = sort_mode;
                last_compute_depth_position_ = camera_position_world;
                last_compute_depth_forward_ = camera_forward_world;
                return UploadSortedIndices(GetView(), result,
                                           *GetRenderMaterial())
                    .Then([this](OwnedTexturePtr sorted_indices_texture)
                              -> absl::Status {
                      // When replacing the texture, we must ensure it
                      // is not destroyed until after the new texture has been
                      // uploaded.
                      // When this block is invoked, the sorted_indices_texture
                      // is guaranteed to be uploaded and ready to use, and the
                      // old texture is no longer referenced by the material.
                      // We can now safely destroy (by overwriting) the previous
                      // sorted_indices_texture_.
                      sorted_indices_texture_ =
                          std::move(sorted_indices_texture);
                      return absl::OkStatus();
                    });
              },
              *state_.sort_in_background ? Executor::Type::kForeground
                                         : Executor::Type::kImmediate);
}

Future<absl::Status> GSplatRenderer::InitializeSortedIndices() {
  const uint2 texture_size = CalculateTextureSize(gsplat_asset_->count());
  sorted_indices_->resize(texture_size.x * texture_size.y);
  std::iota(sorted_indices_->begin(), sorted_indices_->end(), 0);
  // Do an initial sort and upload to make sure we have good data to render.
  SortGaussians(GetCamera());
  return sort_future_;
}

BorrowedMaterialPtr GSplatRenderer::GetRenderMaterial(
    SmallSourceLocation loc) const {
  if (combined_render_and_precompute_material_) {
    return combined_render_and_precompute_material_.Borrow(loc);
  }
  if (render_material_) {
    return render_material_.Borrow(loc);
  }
  return nullptr;
}

BorrowedMaterialPtr GSplatRenderer::GetPrecomputeMaterial(
    SmallSourceLocation loc) const {
  if (combined_render_and_precompute_material_) {
    return combined_render_and_precompute_material_.Borrow(loc);
  }
  if (precompute_material_override_) {
    return precompute_material_override_.WithNewLocation(loc);
  }
  return nullptr;
}

//  Default setup is required for the editor to function.
Future<absl::Status> GSplatRenderer::Setup() { return SetupWithState(); }

Future<absl::Status> GSplatRenderer::Setup(AssetPtr<GSplatAsset> gsplat_asset) {
  gsplat_asset_ = gsplat_asset;
  state_.asset_url = gsplat_asset->url();

  AssetManager& asset_manager = GetView().GetAssetManager();
  Future<AssetPtr<MaterialAsset>> render_material_future =
      asset_manager.LoadMaterial(
          split_engine::GsplatDefaultRenderResource(GetView()),
          MaterialPreCompileOptions{
              .constants = {
                  {.name = std::string(kUseTrianglesForSplatsConstant),
                   .value = *state_.use_triangles_for_splats}}});
  Future<AssetPtr<MaterialAsset>> precompute_material_future =
      asset_manager.LoadMaterial(
          split_engine::kBuiltinGsplatDataPrecomputeMatCmat);

  return render_material_future.Merge(precompute_material_future)
      .Then([this, gsplat_asset](
                std::tuple<AssetPtr<MaterialAsset>, AssetPtr<MaterialAsset>>
                    tuple) mutable -> Future<absl::Status> {
        auto& [render_mat_asset, precompute_mat_asset] = tuple;
        OwnedMaterialPtr render_material = OwnedMaterialPtr(
            GetView().GetMaterialFactory().CreateMaterial(render_mat_asset));
        OwnedMaterialPtr precompute_material =
            OwnedMaterialPtr(GetView().GetMaterialFactory().CreateMaterial(
                precompute_mat_asset));
        return Setup(gsplat_asset, std::move(render_material),
                     std::move(precompute_material));
      });
}

Future<absl::Status> GSplatRenderer::Setup(
    AssetPtr<GSplatAsset> gsplat_asset,
    OwnedMaterialPtr render_and_precompute_material) {
  gsplat_asset_ = gsplat_asset;
  state_.asset_url = gsplat_asset->url();

  combined_render_and_precompute_material_ =
      std::move(render_and_precompute_material);

  return LoadFromResources();
}

Future<absl::Status> GSplatRenderer::Setup(
    AssetPtr<GSplatAsset> gsplat_asset, OwnedMaterialPtr render_material,
    OwnedMaterialPtr precompute_material) {
  gsplat_asset_ = gsplat_asset;
  state_.asset_url = gsplat_asset->url();

  render_material_ = std::move(render_material);

  if (precompute_material) {
    if (GetView().GetSplitEngineSerializer()) {
      return Future<absl::Status>(absl::InvalidArgumentError(
          "Precompute material override is not supported in Split Engine."));
    }

    precompute_node_ = GetNode()->CreateChildNode();
    precompute_node_->SetName("GSplatPrecomputeOverride");

    return precompute_node_
        ->AddComponent<split_engine::PrecomputeTexturePipeline>(
            std::move(precompute_material),
            Box().set(gsplat_asset->bounds_min(), gsplat_asset->bounds_max()))
        .Then([this, gsplat_asset](
                  ComponentHandle<split_engine::PrecomputeTexturePipeline>
                      precompute_texture_pipeline) mutable {
          precompute_texture_pipeline_ = precompute_texture_pipeline;
          absl::Status resize_status =
              precompute_texture_pipeline_->ResizePassTexture(
                  0, gsplat_asset->position_data_texture()->GetSize());
          if (!resize_status.ok()) {
            return Future<absl::Status>(resize_status);
          }
          precompute_material_override_ =
              precompute_texture_pipeline_->BorrowMaterial();

          precompute_material_override_->SetParameter(
              kPositionDataTextureParameter,
              gsplat_asset->position_data_texture(),
              gsplat_asset->position_data_texture()->GetSampler());
          precompute_material_override_->SetParameter(
              kCov3dDataTextureParameter, gsplat_asset->cov3d_data_texture(),
              gsplat_asset->cov3d_data_texture()->GetSampler());
          precompute_material_override_->SetParameter(
              kColorDataTextureParameter, gsplat_asset->color_data_texture(),
              gsplat_asset->color_data_texture()->GetSampler());

          BorrowedTexturePtr precomputed_data_texture =
              precompute_texture_pipeline_->BorrowTexture();
          if (precomputed_data_texture) {
            render_material_->SetParameter(
                kSplatDataPrecomputedParameter, precomputed_data_texture,
                precomputed_data_texture->GetSampler());
          }

          return LoadFromResources();
        });
  }

  return LoadFromResources();
}

Future<absl::Status> GSplatRenderer::SetupWithState() {
  AssetManager& asset_manager = GetView().GetAssetManager();
  Future<AssetPtr<GSplatAsset>> asset_future =
      asset_manager.LoadAsset<GSplatAsset>(*state_.asset_url);
  Future<AssetPtr<MaterialAsset>> render_material_future =
      asset_manager.LoadMaterial(
          split_engine::GsplatDefaultRenderResource(GetView()),
          MaterialPreCompileOptions{
              .constants = {
                  {.name = std::string(kUseTrianglesForSplatsConstant),
                   .value = *state_.use_triangles_for_splats}}});
  Future<AssetPtr<MaterialAsset>> precompute_material_future =
      asset_manager.LoadMaterial(
          split_engine::kBuiltinGsplatDataPrecomputeMatCmat);

  return asset_future.Merge(render_material_future, precompute_material_future)
      .Then([this](std::tuple<AssetPtr<GSplatAsset>, AssetPtr<MaterialAsset>,
                              AssetPtr<MaterialAsset>>
                       tuple) mutable -> Future<absl::Status> {
        auto& [gsplat_asset, render_mat_asset, precompute_mat_asset] = tuple;
        OwnedMaterialPtr render_material(
            GetView().GetMaterialFactory().CreateMaterial(render_mat_asset));
        OwnedMaterialPtr precompute_material(
            GetView().GetMaterialFactory().CreateMaterial(
                precompute_mat_asset));
        return Setup(gsplat_asset, std::move(render_material),
                     std::move(precompute_material));
      });
}

absl::Status GSplatRenderer::UpdateMaterialParameters() {
  BorrowedMaterialPtr render_material = GetRenderMaterial();
  if (!render_material) {
    return absl::InternalError("GSplatRenderer material should not be null.");
  }

  absl::Status status = absl::OkStatus();
  if (state_.min_screen_size->x < 0.f || state_.min_screen_size->y < 0.f) {
    status.Update(absl::InvalidArgumentError(
        "invalid min_screen_size passed to GSplatRenderer"));
  }

  const filament::View* view = GetFilamentView();
  filament::Viewport viewport = view->getViewport();
#if IMP_RUNTIME(DEV)
  // If we are using the default view and an editor viewport override is set,
  // use that instead.
  if (!override_view_) {
    if (auto override_viewport =
            GetView().GetHost()->GetEditorViewportOverride()) {
      viewport = *override_viewport;
    }
  }
#endif
  const filament::DynamicResolutionOptions drs_options =
      view->getDynamicResolutionOptions();
  const float2 scale = drs_options.enabled ? drs_options.maxScale : kOne2;
  const uint2 viewport_size = uint2{viewport.width, viewport.height} * scale;
  const float2 default_max_screen_size = viewport_size * 0.5f;
  const float2 max_screen_size = state_.max_screen_size.HasValue()
                                     ? *state_.max_screen_size
                                     : default_max_screen_size;

  if (state_.max_screen_size->x < 0.f || state_.max_screen_size->y < 0.f) {
    status.Update(absl::InvalidArgumentError(
        "invalid max_screen_size passed to GSplatRenderer"));
  }
  if (max_screen_size.x < state_.min_screen_size->x ||
      max_screen_size.y < state_.min_screen_size->y) {
    status.Update(
        absl::InvalidArgumentError("passed a max_screen_size that was less "
                                   "than min_screen_size to GSplatRenderer"));
  }

  // TODO: (broken link) - Add error checking and unit test for this opacity scale
  // value.
  render_material->SetParameter(kSplatScaleParameter, *state_.splat_scale);

  BorrowedMaterialPtr precompute_material(GetPrecomputeMaterial());
  if (precompute_material) {
    // TODO: (broken link) - Reduce redundant calls to SetParameter. These
    // parameters may be set twice on creation.
    precompute_material->TrySetParameter(kOpacityScaleParameter,
                                         *state_.opacity_scale);
    precompute_material->TrySetParameter(kMinScreenSizeParameter,
                                         *state_.min_screen_size);
    precompute_material->TrySetParameter(kMaxScreenSizeParameter,
                                         max_screen_size);
  }

  if (override_view_) {
    render_material->TrySetParameter(kMainViewResolutionParameter,
                                     viewport_size);
  }
  if (precompute_material_override_) {
    precompute_material_override_->TrySetParameter(kMainViewResolutionParameter,
                                                   viewport_size);
  }
  return status;
}

Future<absl::Status> GSplatRenderer::LoadFromResources() {
  // Only create render component once we have good data to render.
  return InitializeSortedIndices().Then([this]() mutable
                                            -> Future<absl::Status> {
    // We create a vertex buffer with two triangles per splat since we can't use
    // Impress's instancing directly due to its low instance count. Each
    // triangle has 3 vertices so 6 vertices per splat.
    const size_t kVerticesPerSplat = *state_.use_triangles_for_splats ? 3 : 6;
    const size_t vertex_count = gsplat_asset_->count() * kVerticesPerSplat;
    // TODO: (broken link) - Remove the placeholder position attribute once the
    // vulkan backend supports vertex buffers without position attributes.
    MeshDataPtr mesh_data = std::make_unique<MeshData>(MeshDescription{
        .vertex_format = {{VertexFormat::VertexAttribute::POSITION,
                           VertexFormat::AttributeType::FLOAT3}},
        .vertex_count = vertex_count});
    mesh_renderer_ = GetNode()->AddComponent<MeshRenderer>();
    OwnedMeshPtr imp_mesh = GetView().GetMeshFactory().CreateByMovingMeshData(
        MeshFactory::PrimitiveType::TRIANGLES, std::move(mesh_data),
        Box().set(gsplat_asset_->bounds_min(), gsplat_asset_->bounds_max()));
    mesh_renderer_->SetMesh(std::move(imp_mesh));
    mesh_renderer_->SetMaterial(GetRenderMaterial());

    absl::Status status = UpdateMaterialParameters();
    if (!status.ok()) {
      return Future<absl::Status>(status);
    }

    // TODO: (broken link) - Workaround for one-frame delay in rendering. The
    // intention here is to make sure Setup() only completes the future when the
    // Gsplat scene is ready to render
    return Future<absl::Status>::Schedule([]() { return absl::OkStatus(); },
                                          Executor::Type::kForeground);
  });
}

bool GSplatRenderer::IsGsplatAabbOutsideFov(
    imp::ComponentHandle<imp::CameraComponent> camera) const {
  if (!gsplat_asset_) {
    IMP_LOG(imp::WARNING) << "IsGsplatAabbOutsideFov called with null gsplat asset.";
    return false;
  }

  filament::Aabb local_aabb = {gsplat_asset_->bounds_min(),
                               gsplat_asset_->bounds_max()};

  // TODO: (broken link) - Use double precision math for the following
  // AABB-frustum calculations.
  const mat4f& world_trs = GetNode()->GetWorldTrs();
  const filament::Aabb world_aabb = local_aabb.transform(world_trs);

  return camera && camera->GetNode() &&
         !camera->IntersectsFrustum(Box().set(world_aabb.min, world_aabb.max));
}

void GSplatRenderer::Update(const FrameTime& frame_time) {
  IMP_TRACE();
  if (absl::Status result = UpdateMaterialParameters(); !result.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to update Material parameters; " << result.ToString();
  }
  if (precompute_texture_pipeline_ &&
      !precompute_texture_pipeline_->SetCamera(GetCamera()).ok()) {
    IMP_LOG(imp::ERROR) << "Failed to set camera on precompute texture pipeline.";
  }
  MaybeSortGaussians();
}

void GSplatRenderer::Cleanup() {
  if (mesh_renderer_) {
    mesh_renderer_->GetNode()->RemoveComponent<MeshRenderer>();
  }
  if (precompute_texture_pipeline_.IsValid()) {
    BorrowedTexturePtr texture_to_release =
        precompute_texture_pipeline_->BorrowTexture();
    // render_material_ may borrow the texture from the precompute
    // texture pipeline, so must be unset first.
    TryUnsetTexture(GetView(), texture_to_release, render_material_,
                    kSplatDataPrecomputedParameter);
    // combined_render_and_precompute_material_ may borrow the texture
    // from the precompute texture pipeline, so must be unset first.
    TryUnsetTexture(GetView(), texture_to_release,
                    combined_render_and_precompute_material_,
                    kSplatDataPrecomputedParameter);
  }
  if (precompute_material_override_) {
    // precompute material override might borrow the material from the
    // precompute texture pipeline, so must be destroyed first.
    precompute_material_override_ = nullptr;
  }
  if (precompute_texture_pipeline_) {
    precompute_texture_pipeline_->GetNode()
        ->RemoveComponent<split_engine::PrecomputeTexturePipeline>();
  }
  if (precompute_node_) {
    GetView().DestroyNode(precompute_node_);
  }
  combined_render_and_precompute_material_.Reset();
  render_material_.Reset();

  // sort_future_ needs to be destroyed after the render_material_ is destroyed
  // because this future takes ownership of the sorted_indices_texture to upload
  // it, which is borrowed by the render_material_.
  sort_future_.Cancel();

  // The sorted_indices_texture_ is used by render_material_, and thus it
  // needs to be destroyed after the render_material_ is destroyed.
  sorted_indices_texture_.Reset();
}

}  // namespace imp
