/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_CONVEX_HULL_MESH_COLLIDABLE_SHAPE_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_CONVEX_HULL_MESH_COLLIDABLE_SHAPE_H_

#include <memory>
#include <utility>

#include "absl/types/span.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "bullet/src/LinearMath/btAlignedObjectArray.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "bullet/src/LinearMath/btVector3.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/collidable_shapes/collidable_shape.h"

namespace imp {

// This class makes a convex hull of the mesh and uses it as a collidable shape.
class ConvexHullMeshCollidableShape : public CollidableShape {
 public:
  explicit ConvexHullMeshCollidableShape(NodeHandle node);

  void CreateBtCollisionShape() override;

  btCollisionShape* GetBtCollisionShape() const override;

  float3 GetCollidableCenter() const override;

  CollidableShape::CollisionShape GetCollisionShape(
      const btTransform& bt_trans) const override;

  void ApplyScalingToBulletCollider() override {}

#if IMP_RUNTIME(DEV)
  void Visualize(const btTransform& bt_trans) const override;
#endif

 private:
  NodeHandle node_;
  btAlignedObjectArray<btVector3> vertices_;
  btCollisionShape* collidable_shape_;
  std::unique_ptr<btConvexHullShape> convex_hull_shape_;

  absl::Span<const MeshVertexAndIndexData> mesh_data_;

  void PushVertices(MeshVertexData* vertex_data, MeshIndexData* index_data);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_CONVEX_HULL_MESH_COLLIDABLE_SHAPE_H_
