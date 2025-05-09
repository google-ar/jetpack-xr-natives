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
#include <utility>

#include "absl/types/span.h"
#include "bullet/src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/config.h"
#include "core/geometry/shapes/box.h"
#include "core/math/vec.h"
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
#if IMP_RUNTIME(DEV)
#include "core/common/debug_draw.h"
#endif

namespace imp {
namespace {

float3 GetVertexWorldPosition(MeshVertexAndIndexData mesh, size_t index,
                              NodeHandle node) {
  MeshVertexData* vertex_data = mesh.vertex_data;
  MeshIndexData* index_data = mesh.index_data;
  uint32_t id = index_data->GetDescription().index_type ==
                        MeshDescription::IndexType::USHORT
                    ? index_data->IndexAt<uint16_t>(index)
                    : index_data->IndexAt<uint32_t>(index);
  return node->WorldFromLocalPoint(vertex_data->VertexAttributeAt<float3>(
      id, VertexFormat::VertexAttribute::POSITION));
}
}  // namespace

StaticMeshCollidableShape::StaticMeshCollidableShape(NodeHandle node)
    : node_(node) {}

btTransform StaticMeshCollidableShape::AddBtCollisionShape() {
  ComponentHandle<GltfMesh> mesh = node_->GetComponent<GltfMesh>();
  // TODO: Make sure the meshes are watertight. Tips: Use Blender
  // to apply voxel remesh to the asset to make it watertight.
  mesh_data_ = mesh->GetMeshData();

  for (const MeshVertexAndIndexData& data : mesh_data_) {
    MeshDescription index_description = data.index_data->GetDescription();
    for (size_t i = 0; i < index_description.index_count / 3; i++) {
      float3 p0 = GetVertexWorldPosition(data, i * 3, node_);
      float3 p1 = GetVertexWorldPosition(data, i * 3 + 1, node_);
      float3 p2 = GetVertexWorldPosition(data, i * 3 + 2, node_);

      triangle_mesh_.addTriangle(ToBtVector3(p0), ToBtVector3(p1),
                                 ToBtVector3(p2));
    }
  }
  collidable_shape_ =
      std::make_unique<btBvhTriangleMeshShape>(&triangle_mesh_, true, true);
  world_bounds_ = mesh->GetLocalBounds();

  btTransform transform;
  transform.setIdentity();

  return transform;
}

btCollisionShape* StaticMeshCollidableShape::GetCollidableShape() const {
  return collidable_shape_.get();
}

float3 StaticMeshCollidableShape::GetCollidableCenter() const {
  // static mesh doesn't support offsets.
  return float3(0.0f);
}

CollidableShape::CollisionShape StaticMeshCollidableShape::GetCollisionShape(
    const btTransform& transform) const {
  return world_bounds_;
}

#if IMP_RUNTIME(DEV)
void StaticMeshCollidableShape::Visualize(const btTransform& transform) const {
  debug_draw::Local(node_.GetEntity())
      .BoxLines(world_bounds_,
                debug_draw::GetColor(debug_draw::DebugColor::kDeepOrange));
  debug_draw::Local(node_.GetEntity())
      .MeshLines(mesh_data_,
                 debug_draw::GetColor(debug_draw::DebugColor::kDeepOrange));
}
#endif

}  // namespace imp
