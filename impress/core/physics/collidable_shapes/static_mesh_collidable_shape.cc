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

#include "core/physics/collidable_shapes/static_mesh_collidable_shape.h"

#include <cstddef>
#include <cstdint>
#include <memory>

#include "absl/types/span.h"
#include "bullet/src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/common/filament_helpers.h"
#include "core/config.h"
#include "core/geometry/shapes/box.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/collidable_shapes/collidable_shape.h"
#include "core/physics/physics_helper.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/render/mesh_renderer.h"
#if IMP_RUNTIME(DEV)
#include "core/common/debug_draw.h"
#endif

namespace imp {
namespace {

float3 GetVertexWorldPosition(MeshVertexData* vertex_data,
                              MeshIndexData* index_data, size_t index,
                              NodeHandle node) {
  uint32_t id = index_data->GetDescription().index_type ==
                        MeshDescription::IndexType::USHORT
                    ? index_data->IndexAt<uint16_t>(index)
                    : index_data->IndexAt<uint32_t>(index);
  return node->WorldFromLocalPoint(vertex_data->VertexAttributeAt<float3>(
      id, VertexFormat::VertexAttribute::POSITION));
}

}  // namespace

StaticMeshCollidableShape::StaticMeshCollidableShape(NodeHandle node)
    : CollidableShape(node) {}

void StaticMeshCollidableShape::PushTriangles(MeshVertexData* vertex_data,
                                              MeshIndexData* index_data) {
  auto index_count = index_data->GetDescription().index_count;
  for (int i = 0; i < index_count; i += 3) {
    auto vertex0 = GetVertexWorldPosition(vertex_data, index_data, i, node_);
    auto vertex1 =
        GetVertexWorldPosition(vertex_data, index_data, i + 1, node_);
    auto vertex2 =
        GetVertexWorldPosition(vertex_data, index_data, i + 2, node_);
    triangle_mesh_.addTriangle(ToBtVector3(vertex0), ToBtVector3(vertex1),
                               ToBtVector3(vertex2));
  }
}

void StaticMeshCollidableShape::CreateBtCollisionShape() {
  ComponentHandle<GltfMesh> gltf_mesh = node_->GetComponent<GltfMesh>();
  if (gltf_mesh.IsValid()) {
    // TODO: Make sure the meshes are watertight. Tips: Use Blender
    // to apply voxel remesh to the asset to make it watertight.
    mesh_data_ = gltf_mesh->GetMeshData();

    for (const MeshVertexAndIndexData& data : mesh_data_) {
      PushTriangles(data.vertex_data, data.index_data);
    }
    world_bounds_ =
        TransformBounds(gltf_mesh->GetLocalBounds(), node_->GetWorldTrs());
  } else {
    ComponentHandle<MeshRenderer> mesh_renderer =
        node_->GetComponent<MeshRenderer>();
    if (mesh_renderer.IsValid()) {
      for (size_t i = 0; i < mesh_renderer->GetPrimitiveCount(); ++i) {
        Mesh* mesh = mesh_renderer->GetMesh(i);
        if (!mesh) continue;
        MeshData* mesh_data = mesh->GetMeshData();
        if (!mesh_data) continue;
        PushTriangles(mesh_data->GetVertexData(), mesh_data->GetIndexData());
      }
      world_bounds_ = TransformBounds(mesh_renderer->GetRenderableAabb(),
                                      node_->GetWorldTrs());
    }
  }
  collidable_shape_ =
      std::make_unique<btBvhTriangleMeshShape>(&triangle_mesh_, true, true);
}

btCollisionShape* StaticMeshCollidableShape::GetBtCollisionShape() const {
  return collidable_shape_.get();
}

float3 StaticMeshCollidableShape::GetCollidableCenter() const {
  // static mesh doesn't support offsets.
  return float3(0.0f);
}

CollidableShape::CollisionShape StaticMeshCollidableShape::GetCollisionShape(
    const btTransform& bt_trans) const {
  return world_bounds_;
}

#if IMP_RUNTIME(DEV)
void StaticMeshCollidableShape::Visualize(const btTransform& bt_trans) const {
  debug_draw::Global().BoxLines(
      world_bounds_, debug_draw::GetColor(debug_draw::DebugColor::kDeepOrange));
  debug_draw::Global().MeshLines(
      mesh_data_, debug_draw::GetColor(debug_draw::DebugColor::kDeepOrange));
}
#endif

}  // namespace imp
