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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_H_

#include <memory>

#include "absl/status/status.h"
#include "bullet/src/BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/collidable_shapes/collidable_shape.h"
#include "core/physics/collidable_type.proto.imp.h"

namespace imp {

// This is a wrapper class that helps link a Bullet collider of correct shape to
// corresponding the existing Impress collider, so that to hide the complexity
// of dealing with different Bullet collision shapes from the user.
//
// It also lists methods that queries and modified underlying Bullet collision
// shapes. Those methods are used in several physics components (e.g. RigidBody,
// TriggerVolume)
class Collidable {
 public:
  using CollisionShape = CollidableShape::CollisionShape;

  // Matches Impress collider with a Bullet collider.
  absl::Status Create(btTransform& start_transform, NodeHandle node,
                      physics::CollidableType type =
                          physics::CollidableType::AUTOMATIC_DEFAULT);

  // (If returns true) Indicates that this collidable should not be transformed,
  // so as the corresponding node.
  bool IsMovable() const { return is_movable_; }

  // Returns a Bullet collision shape.
  btCollisionShape* GetCollidableShape();

  float3 GetCollidableCenter();

  physics::CollidableType GetCollidableType() const { return type_; }

  // Returns the position and size (but not orientation) of the collision shape,
  // for testing the alignment between Impress and Bullet colliders.
  //
  // Takes the transformation of the instance of btCollisionObject (e.g.
  // btRigidBody, btGhostObject) in subclass as the input.
  CollisionShape GetCollisionShape(const btTransform& transform) const;

  void ApplyScalingToBulletCollider();

#if IMP_RUNTIME(DEV)
  // Takes the transformation of the instance of btCollisionObject (e.g.
  // btRigidBody, btGhostObject) in subclass as the input.
  void Visualize(const btTransform& transform) const;
#endif

 private:
  std::unique_ptr<CollidableShape> collidable_shape_;
  bool is_movable_ = true;
  physics::CollidableType type_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_H_
