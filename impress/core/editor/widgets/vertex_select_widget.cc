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

#include "core/editor/widgets/vertex_select_widget.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/collision/collision_helpers.h"
#include "core/common/debug_draw.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/ncsb/update_system.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp::editor {

using SampledJointId = model::ModelData::SampledJointId;
using JointId = model::ModelData::JointId;
using BoneId = model::BoneId;
using SkinData = model::ModelData::SkinData;
using CollidedTriangle = collision::CollidedTriangle;

// TODO : Remove this after the fix.
constexpr float kDisplaySphereSizeMeters = 0.003f;

VertexSelectWidget::VertexSelectWidget(BaseView& view, Dispatcher& dispatcher,
                                       NodeHandle editor_root_node)
    : view_(view),
      dispatcher_(dispatcher),
      editor_root_node_(editor_root_node) {
  // Find the closest hit target when the mouse hooks over.
  dispatcher_.Connect(
      [this](const PointerHitEvent& event) {
        if (event.GetPointerCount() != 1) {
          active_node_ = NodeHandle();
          return;
        }
        double3 collision_point;
        for (size_t i = 0; i < event.GetAllIntersectingNodes(0).size(); ++i) {
          NodeHandle node = event.GetAllIntersectingNodes(0)[i];
          // Skip nodes that are part of the editor.
          if (view_.GetPathManager().IsAncestorOf(editor_root_node_, node)) {
            continue;
          }
          if (absl::holds_alternative<std::vector<std::vector<RayHit>>>(
                  event.hits)) {
            RayHit ray_hit =
                absl::get<std::vector<std::vector<RayHit>>>(event.hits)[0][i];
            auto* col_triangle = ray_hit.GetMetaData<CollidedTriangle>();

            if (!col_triangle) {
              continue;
            }

            active_node_ = node;
            picked_triangle_ = *col_triangle;
            collision_point = static_cast<double3>(ray_hit.world_point);

            auto vertex_position = *GetVertexPositionsPrecise();
            picked_vertex_id_ = 0;
            for (size_t p = 0; p < 3; p++) {
              if (norm(vertex_position[p] - collision_point) <
                  norm(vertex_position[picked_vertex_id_] - collision_point)) {
                picked_vertex_id_ = p;
              }
            }
            return;
          } else {
            DoubleRayHit ray_hit =
                absl::get<std::vector<std::vector<DoubleRayHit>>>(
                    event.hits)[0][i];
            auto* col_triangle = ray_hit.GetMetaData<CollidedTriangle>();

            if (!col_triangle) {
              continue;
            }

            active_node_ = node;
            picked_triangle_ = *col_triangle;
            collision_point = ray_hit.world_point;

            auto vertex_position = *GetVertexPositionsPrecise();
            picked_vertex_id_ = 0;
            for (size_t p = 0; p < 3; ++p) {
              if (norm(vertex_position[p] - collision_point) <
                  norm(vertex_position[picked_vertex_id_] - collision_point)) {
                picked_vertex_id_ = p;
              }
            }
            return;
          }
        }
        active_node_ = NodeHandle();
      },
      this);

  // Draw vertex selection and update meshes.
  dispatcher_.Connect(
      [this](const UpdateSystem::PostComponentsUpdateEvent& event) {
        if (active_node_) {
          Draw();
        }
      },
      this);
}

absl::optional<std::array<double3, 3>>
VertexSelectWidget::GetVertexPositionsPrecise() {
  if (!active_node_) {
    return absl::nullopt;
  }
  auto primitive = active_node_->GetComponent<GltfMesh>()
                       ->GetMeshData()[picked_triangle_->primitive_id];
  MeshVertexData* vertices = primitive.vertex_data;
  MeshIndexData* indices = primitive.index_data;

  auto get_world_position = [vertices, indices,
                             node = active_node_](size_t id) {
    uint32_t index = indices->GetDescription().index_type ==
                             MeshDescription::IndexType::USHORT
                         ? indices->IndexAt<uint16_t>(id)
                         : indices->IndexAt<uint32_t>(id);
    return node->WorldFromLocalPointPrecise(vertices->VertexAttributeAt<float3>(
        index, VertexFormat::VertexAttribute::POSITION));
  };

  std::array<double3, 3> points;
  points[0] = get_world_position(picked_triangle_->triangle_id * 3);
  points[1] = get_world_position(picked_triangle_->triangle_id * 3 + 1);
  points[2] = get_world_position(picked_triangle_->triangle_id * 3 + 2);

  return points;
}

void VertexSelectWidget::Draw() {
  auto vertex_position_optional = GetVertexPositionsPrecise();
  if (!vertex_position_optional.has_value()) {
    return;
  }
  auto vertex_position = *vertex_position_optional;

  auto draw_line = [&](double3 start, double3 end) {
    debug_draw::Global().Line(start, end, debug_draw::kLightGreen);
  };
  // The collision triangle.
  draw_line(vertex_position[0], vertex_position[1]);
  draw_line(vertex_position[1], vertex_position[2]);
  draw_line(vertex_position[2], vertex_position[0]);

  // The candidate vertex.
  debug_draw::Global().SphereLines(vertex_position[picked_vertex_id_],
                                   kDisplaySphereSizeMeters, debug_draw::kRed);
}

}  // namespace imp::editor
