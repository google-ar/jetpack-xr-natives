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

#include "core/render/degenerate_triangle_renderer_for_driver_bug_workaround.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/geometry/shapes/box.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_factory.h"
#include "core/model/mesh/vertex_format.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/render/invisible_material_assets.h"
#include "core/render/mesh_renderer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"

namespace imp {

Future<absl::Status> DegenerateTriangleRendererForDriverBugWorkaround::Setup() {
  BaseView& view = GetView();
  degenerate_triangle_node_ = GetNode()->CreateChildNode();
  degenerate_triangle_node_->SetName("BuiltinGSplatDegenerateTriangle");
  degenerate_triangle_renderer_ =
      degenerate_triangle_node_->AddComponent<MeshRenderer>(
          MeshRenderer::FrustumCullingMode::kDisabled);

  MeshDataPtr mesh_data = std::make_unique<MeshData>(MeshDescription{
      .vertex_format = {{VertexFormat::VertexAttribute::POSITION,
                         VertexFormat::AttributeType::FLOAT3}},
      .index_type = MeshDescription::IndexType::USHORT,
      .vertex_count = 3,
      .index_count = 3});
  absl::Span<float3> vertices = mesh_data->Vertices<float3>();
  std::fill(vertices.begin(), vertices.end(), float3(0.0f));
  absl::Span<uint16_t> indices = mesh_data->Indices<uint16_t>();
  indices[0] = 0;
  indices[1] = 1;
  indices[2] = 2;

  OwnedMeshPtr mesh = view.GetMeshFactory().CreateByMovingMeshData(
      MeshFactory::PrimitiveType::TRIANGLES, std::move(mesh_data),
      /*aabb=*/Box{});
  degenerate_triangle_renderer_->SetMesh(std::move(mesh));

  return view.GetAssetManager()
      .LoadMaterial(render::kInvisibleMaterialCmat)
      .Then([this, &view](AssetPtr<MaterialAsset> invisible_material_asset)
                -> absl::Status {
        if (!invisible_material_asset) {
          return absl::InternalError("Failed to load invisible material.");
        }
        if (!degenerate_triangle_renderer_) {
          return absl::InternalError(
              "degenerate_triangle_renderer_ was cleaned up before material "
              "loaded");
        }
        degenerate_triangle_renderer_->SetMaterial(
            view.GetMaterialFactory().CreateMaterial(invisible_material_asset));
        return absl::OkStatus();
      });
}

void DegenerateTriangleRendererForDriverBugWorkaround::Cleanup() {
  if (degenerate_triangle_node_) {
    GetView().DestroyNode(degenerate_triangle_node_);
  }
}

}  // namespace imp
