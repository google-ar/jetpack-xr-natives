// Copyright 2024 Google LLC
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

#include "core/line_renderer/line_renderer.h"

#include <tuple>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "core/line_renderer/assets.h"
#include "core/line_renderer/line_buffer.h"
#include "core/line_renderer/line_extruder_3d.h"
#include "core/line_renderer/polyline.h"
#include "core/line_renderer/stroke_polyline_util.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/view/framework/render/mesh_renderer.h"

namespace imp {

imp::Future<absl::Status> LineRenderer::Setup() {
  std::vector<std::tuple<float3, float3>> points_with_normals;
  points_with_normals.reserve(state_.points.size());
  for (const float3& point : state_.points) {
    points_with_normals.emplace_back(point, state_.normal);
  }
  line_renderer::Polyline3f polyline(points_with_normals.begin(),
                                     points_with_normals.end());
  line_renderer::LineBuffer line_buffer(
      line_renderer::MaxVerticesForExtrudedStroke(
          state_.points.size(), /*has_stamp=*/false, state_.start_cap_shape,
          state_.end_cap_shape, state_.joint_shape,
          /*connect_to_previous=*/false,
          /*loop=*/state_.wrap),
      line_renderer::MaxIndicesForExtrudedStroke(
          state_.points.size(), state_.start_cap_shape, state_.end_cap_shape,
          state_.joint_shape,
          /*connect_to_previous=*/false,
          /*loop=*/state_.wrap),
      1.0f, 1.0f);

  line_renderer::ExtrudeLine3D(line_buffer, polyline, state_.joint_shape,
                               state_.start_cap_shape, state_.end_cap_shape,
                               false,
                               /*style_index*/ 0, /*zoom_range*/ 0,
                               /*start_distance=*/0.0f,
                               /*total_unclipped_distance=*/0.0f,
                               /*connect_to_previous=*/false,
                               /*loop_self=*/state_.wrap, /*loop_first=*/false,
                               /*orthogonal_offset_scale=*/1.0f,
                               /*position=*/{0.0f, 0.0f, 0.0f});
  MeshDataPtr mesh_data = line_buffer.FinalizeAndReleaseMeshData();
  OwnedMeshPtr mesh_ptr = GetView().GetMeshFactory().CreateByMovingMeshData(
      MeshFactory::PrimitiveType::TRIANGLES, std::move(mesh_data),
      line_buffer.GetBoundingBox());
  ComponentHandle<MeshRenderer> mesh_renderer =
      GetNode()->AddComponent<MeshRenderer>();
  mesh_renderer->SetMesh(std::move(mesh_ptr));
  return GetView()
      .GetAssetManager()
      .LoadMaterial(line_renderer::kLine3dCmat)
      .Then([this](AssetPtr<MaterialAsset> material) {
        OwnedMaterialPtr material_ptr =
            GetView().GetMaterialFactory().CreateMaterial(material);
        GetNode()->GetComponent<MeshRenderer>()->SetMaterial(
            std::move(material_ptr));
        OnIsfStateChanged();
        return absl::OkStatus();
      });
}

void LineRenderer::SetColor(const float4& color) {
  state_.color = color;
  OnIsfStateChanged();
}

void LineRenderer::OnIsfStateChanged() {
  ComponentHandle<MeshRenderer> mesh_renderer =
      GetNode()->GetComponent<MeshRenderer>();
  Material* material = mesh_renderer->GetMaterial();
  material->SetParameter("color", state_.color);
  material->SetParameter("width", state_.width);
  material->SetParameter("feather", state_.feather);
  material->SetParameter("zoom", 1.0f);
}

}  // namespace imp
