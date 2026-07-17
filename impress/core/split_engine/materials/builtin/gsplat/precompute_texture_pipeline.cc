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

#include "core/split_engine/materials/builtin/gsplat/precompute_texture_pipeline.h"

#include <optional>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/box.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/render/mesh_renderer.h"
#include "core/render/texture.h"
#include "core/render_passes/texture_pipeline_renderer.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/resources/resource_definition.h"
#include "core/scene_handles/scene_handles.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"

namespace imp::split_engine {

namespace {
// An offscreen quad for the precompute pass.
imp::ComponentHandle<imp::MeshRenderer> CreateMeshRenderer(
    NodeHandle precompute_node, imp::BorrowedMaterialPtr precompute_material,
    std::optional<imp::Box> aabb_override) {
  const std::string texture_group_name =
      absl::StrCat(precompute_node.GetEntity().getId());
  precompute_node->SetGroups({texture_group_name});
  auto culling_mode = aabb_override
                          ? imp::MeshRenderer::FrustumCullingMode::kEnabled
                          : imp::MeshRenderer::FrustumCullingMode::kDisabled;
  imp::ComponentHandle<imp::MeshRenderer> mesh_renderer =
      precompute_node->AddComponent<imp::MeshRenderer>(culling_mode);

  imp::OwnedMeshPtr mesh =
      precompute_node->GetView().GetMeshFactory().CreateQuad();
  if (aabb_override.has_value()) {
    mesh->AssignAabb(*aabb_override);
  }
  mesh_renderer->SetMesh(std::move(mesh));

  mesh_renderer->SetShadowCastingMode(imp::MeshRenderer::ShadowMode::kNone);
  mesh_renderer->SetShadowReceivingMode(imp::MeshRenderer::ShadowMode::kNone);
  mesh_renderer->SetMaterial(std::move(precompute_material));
  return mesh_renderer;
}

// Configuration for the precompute pass
imp::TexturePipelineRendererState::Pass ConfigurePrecomputeDataTexture(
    const std::string& texture_group_name, uint2 size,
    imp::ComponentHandle<imp::CameraComponent> camera = {}) {
  return {
      .group = texture_group_name,
      .camera = camera ? ComponentSceneHandle<imp::CameraComponent>(camera)
                       : ComponentSceneHandle<imp::CameraComponent>(),
      .color_texture_config =
          imp::TexturePipelineRendererState::Texture{
              .name = texture_group_name,
              .format = imp::TexturePipelineRendererState::Texture::RGBA32UI,
          },
      .texture_size = size,
      .render_settings =
          imp::render_settings::ViewRenderSettings{.post_processing_enabled =
                                                       false},
      .use_main_view_settings = true,
  };
}

// Constructs a TexturePipelineRenderer which will render the precompute data
// texture. The group name is based on node id and will be used in the render
// pass
imp::Future<ComponentHandle<imp::TexturePipelineRenderer>>
CreateTexturePipelineRenderer(
    NodeHandle precompute_node, uint2 size,
    imp::ComponentHandle<imp::CameraComponent> camera = {}) {
  const std::string texture_group_name =
      absl::StrCat(precompute_node.GetEntity().getId());

  imp::TexturePipelineRendererState texture_pipeline_renderer_state = {
      .passes = {ConfigurePrecomputeDataTexture(texture_group_name, size,
                                                camera)},
      // We give a high priority to this precompute pass, since it needs to run
      // before other passes, especially when Gsplat is set up to render to
      // another offscreen texture.
      .priority = 10,
  };

  return precompute_node->AddComponentWithState<imp::TexturePipelineRenderer>(
      texture_pipeline_renderer_state);
}
}  // namespace

imp::Future<absl::Status> PrecomputeTexturePipeline::Setup() {
  return absl::UnimplementedError(
      "PrecomputeTexturePipeline does not support parameterless setup.");
}

imp::Future<absl::Status> PrecomputeTexturePipeline::Setup(
    resources::ResourceDefinition precompute_material_definition,
    std::optional<imp::Box> aabb_override) {
  imp::BaseView& view = GetView();

  return view.GetAssetManager()
      .LoadMaterial(precompute_material_definition)
      .Then([this, aabb_override](AssetPtr<MaterialAsset> material_asset)
                -> imp::Future<absl::Status> {
        OwnedMaterialPtr precompute_material(
            GetView().GetMaterialFactory().CreateMaterial(material_asset));
        return Setup(std::move(precompute_material), aabb_override);
      });
}

imp::Future<absl::Status> PrecomputeTexturePipeline::Setup(
    imp::OwnedMaterialPtr precompute_material,
    std::optional<imp::Box> aabb_override) {
  precompute_material_ = std::move(precompute_material);
  imp::NodeHandle precompute_node = GetNode();

  mesh_renderer_ = CreateMeshRenderer(
      precompute_node, precompute_material_.Borrow(), aabb_override);

  return CreateTexturePipelineRenderer(precompute_node, size_, camera_override_)
      .Then([this](ComponentHandle<TexturePipelineRenderer> renderer) {
        texture_pipeline_renderer_ = renderer;
        return absl::OkStatus();
      });
}

void PrecomputeTexturePipeline::Cleanup() {
  // Cleanup components which this class added
  GetNode()->RemoveComponent<imp::MeshRenderer>();
  GetNode()->RemoveComponent<imp::TexturePipelineRenderer>();
}

void PrecomputeTexturePipeline::OnActiveStatusChanged(bool is_active) {
  texture_pipeline_renderer_->SetPassEnabled(0, is_active);
}

BorrowedMaterialPtr PrecomputeTexturePipeline::BorrowMaterial(
    SmallSourceLocation loc) const {
  return precompute_material_.Borrow(loc);
}

BorrowedTexturePtr PrecomputeTexturePipeline::BorrowTexture(
    SmallSourceLocation loc) const {
  return GetView().GetTextureRegistry().BorrowTexture(
      absl::StrCat(GetNode().GetEntity().getId()), loc);
}

absl::Status PrecomputeTexturePipeline::ResizePassTexture(
    int pass_index, imp::uint2 texture_size) {
  if (size_ == texture_size) {
    return absl::OkStatus();
  }

  size_ = texture_size;
  return texture_pipeline_renderer_->ResizePassTexture(pass_index,
                                                       texture_size);
}

absl::Status PrecomputeTexturePipeline::SetCamera(
    imp::ComponentHandle<imp::CameraComponent> camera) {
  camera_override_ = camera;
  return texture_pipeline_renderer_->SetPassCamera(0, camera_override_);
}

}  // namespace imp::split_engine
