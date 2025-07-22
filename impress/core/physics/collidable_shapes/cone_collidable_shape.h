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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_CONE_COLLIDABLE_SHAPE_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_CONE_COLLIDABLE_SHAPE_H_

#include <memory>

#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/collidable_shapes/collidable_shape.h"

namespace imp {

// This class deals with Bullet cone collision shapes.
class ConeCollidableShape : public CollidableShape {
 public:
  ConeCollidableShape(NodeHandle node);

  void CreateBtCollisionShape() override;

  btCollisionShape* GetBtCollisionShape() const override;

  float3 GetCollidableCenter() const override;

  CollidableShape::CollisionShape GetCollisionShape(
      const btTransform& bt_trans) const override;

  void ApplyScalingToBulletCollider() override;

#if IMP_RUNTIME(DEV)
  void Visualize(const btTransform& bt_trans) const override;
#endif

 private:
  // Since btConeShape does not support non-uniform scaling, this function
  // forces the three dimensions of the scale of an Impress cone collider to
  // be the same.
  void EnforceEvenScaleForCone();

  NodeHandle node_;
  std::unique_ptr<btCollisionShape> collidable_shape_;
  // Local offset of the center of the cone in Impress.
  float3 collidable_center_;

  float3 scale_prev_ = float3(1.0f);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_CONE_COLLIDABLE_SHAPE_H_
