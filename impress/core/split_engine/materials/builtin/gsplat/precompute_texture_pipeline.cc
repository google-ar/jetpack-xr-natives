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

#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/render/texture.h"
#include "core/render_passes/texture_pipeline_renderer.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/split_engine/materials/builtin/gsplat/gsplat_material_deserializer_assets.h"
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
    NodeHandle precompute_node, imp::BorrowedMaterialPtr precompute_material) {
  const std::string texture_group_name =
      absl::StrCat(precompute_node.GetEntity().getId());
  precompute_node->SetName("ParticleRendererDataPrecompute");
  precompute_node->SetGroups({texture_group_name});
  imp::ComponentHandle<imp::MeshRenderer> mesh_renderer =
      precompute_node->AddComponent<imp::MeshRenderer>(
          imp::MeshRenderer::FrustumCullingMode::kDisabled);
  mesh_renderer->SetMesh(
      precompute_node->GetView().GetMeshFactory().CreateQuad());
  mesh_renderer->SetShadowCastingMode(imp::MeshRenderer::ShadowMode::kNone);
  mesh_renderer->SetShadowReceivingMode(imp::MeshRenderer::ShadowMode::kNone);
  mesh_renderer->SetMaterial(std::move(precompute_material));
  return mesh_renderer;
}

// Configuration for the precompute pass
imp::TexturePipelineRendererState::Pass ConfigurePrecomputeDataTexture(
    const std::string& texture_group_name, uint2 size) {
  return {
      .group = texture_group_name,
      .color_texture_config =
          imp::TexturePipelineRendererState::Texture{
              .name = texture_group_name,
              .format = imp::TexturePipelineRendererState::Texture::RGBA32F,
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
CreateTexturePipelineRenderer(NodeHandle precompute_node, uint2 size) {
  const std::string texture_group_name =
      absl::StrCat(precompute_node.GetEntity().getId());

  imp::TexturePipelineRendererState texture_pipeline_renderer_state = {
      .passes = {ConfigurePrecomputeDataTexture(texture_group_name, size)}};

  return precompute_node->AddComponentWithState<imp::TexturePipelineRenderer>(
      texture_pipeline_renderer_state);
}
}  // namespace

imp::Future<absl::Status> PrecomputeTexturePipeline::Setup() {
  imp::BaseView& view = GetView();
  imp::NodeHandle precompute_node = GetNode();

  return view.GetAssetManager()
      .LoadMaterial(kBuiltinGsplatDataPrecomputeMatCmat)
      .Then([this](AssetPtr<MaterialAsset> material_asset) -> absl::Status {
        precompute_material_ = OwnedMaterialPtr(
            GetView().GetMaterialFactory().CreateMaterial(material_asset));
        return absl::OkStatus();
      })
      .Then([this, precompute_node](absl::Status status) mutable {
        mesh_renderer_ =
            CreateMeshRenderer(precompute_node, precompute_material_.Borrow());
        return absl::OkStatus();
      })
      .Then([this](absl::Status status) mutable {
        return CreateTexturePipelineRenderer(GetNode(), size_)
            .Then([this](ComponentHandle<TexturePipelineRenderer> renderer) {
              texture_pipeline_renderer_ = renderer;
              return absl::OkStatus();
            });
      });
}

void PrecomputeTexturePipeline::Cleanup() {
  // Cleanup components which this class added
  GetNode()->RemoveComponent<imp::MeshRenderer>();
  GetNode()->RemoveComponent<imp::TexturePipelineRenderer>();
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

}  // namespace imp::split_engine
