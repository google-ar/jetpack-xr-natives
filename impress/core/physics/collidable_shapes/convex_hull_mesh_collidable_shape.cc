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

#include "core/physics/collidable_shapes/convex_hull_mesh_collidable_shape.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "absl/types/span.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "bullet/src/BulletCollision/CollisionShapes/btShapeHull.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "bullet/src/LinearMath/btVector3.h"
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

ConvexHullMeshCollidableShape::ConvexHullMeshCollidableShape(NodeHandle node)
    : node_(node) {}

btTransform ConvexHullMeshCollidableShape::AddBtCollisionShape() {
  ComponentHandle<GltfMesh> mesh = node_->GetComponent<GltfMesh>();
  mesh_data_ = mesh->GetMeshData();

  size_t num_vertices = 0;
  for (const MeshVertexAndIndexData& data : mesh_data_) {
    MeshDescription index_description = data.index_data->GetDescription();
    num_vertices += index_description.index_count / 3 * 3;
    for (size_t i = 0; i < index_description.index_count / 3; i++) {
      float3 p0 = GetVertexWorldPosition(data, i * 3, node_);
      float3 p1 = GetVertexWorldPosition(data, i * 3 + 1, node_);
      float3 p2 = GetVertexWorldPosition(data, i * 3 + 2, node_);

      vertices_.push_back(ToBtVector3(p0));
      vertices_.push_back(ToBtVector3(p1));
      vertices_.push_back(ToBtVector3(p2));
    }
  }

  btConvexHullShape convexHullShape(&vertices_[0].x(), vertices_.size(),
                                    sizeof(btVector3));

  // Create a hull approximation
  btShapeHull hull = btShapeHull(&convexHullShape);
  hull.buildHull(0);  // note: parameter is ignored by buildHull

  const btVector3* hull_vertices = hull.getVertexPointer();
  convex_hull_shape_ = std::make_unique<btConvexHullShape>(
      &hull_vertices->getX(), hull.numVertices(), sizeof(btVector3));
  collidable_shape_ = convex_hull_shape_.get();

  btTransform transform;
  transform.setIdentity();

  return transform;
}

btCollisionShape* ConvexHullMeshCollidableShape::GetCollidableShape() const {
  return collidable_shape_;
}

float3 ConvexHullMeshCollidableShape::GetCollidableCenter() const {
  // Mesh doesn't support offset.
  return float3(0.0f);
}

CollidableShape::CollisionShape
ConvexHullMeshCollidableShape::GetCollisionShape(
    const btTransform& transform) const {
  // TODO: Implement this.
  return Box();
}

#if IMP_RUNTIME(DEV)
void ConvexHullMeshCollidableShape::Visualize(
    const btTransform& transform) const {
  for (size_t i = 0; i < convex_hull_shape_->getNumEdges(); i++) {
    btVector3 v0;
    btVector3 v1;
    convex_hull_shape_->getEdge(i, v0, v1);
    debug_draw::Local(node_.GetEntity())
        .Line(ToVec3<float>(v0), ToVec3<float>(v1),
              debug_draw::GetColor(debug_draw::DebugColor::kDeepOrange));
  }
}
#endif

}  // namespace imp
