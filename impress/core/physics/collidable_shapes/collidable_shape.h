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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_COLLIDABLE_SHAPE_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_COLLIDABLE_SHAPE_H_

#include <variant>

#include "absl/types/optional.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/config.h"
#include "core/geometry/shapes/box.h"
#include "core/geometry/shapes/capsule.h"
#include "core/geometry/shapes/compound_shape.h"
#include "core/geometry/shapes/cone.h"
#include "core/geometry/shapes/cylinder.h"
#include "core/geometry/shapes/sphere.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"

namespace imp {

// This is a interface to be used in implementing methods that queries/modifies
// Bullet collision shapes of different types.
class CollidableShape {
 public:
  using CollisionShape = absl::optional<
      std::variant<Sphere, Box, Capsule, Cylinder, Cone, CompoundShape>>;

  CollidableShape(NodeHandle node);
  virtual ~CollidableShape() = default;

  // Creates a Bullet collision shape that matches the Impress collider's shape.
  virtual void CreateBtCollisionShape() = 0;

  // Gets the shape of the Bullet collision shape for tests.
  virtual btCollisionShape* GetBtCollisionShape() const = 0;

  // Gets the center of the Bullet collision shape.
  virtual float3 GetCollidableCenter() const = 0;

  // Returns the position and size (but not orientation) of the collision shape,
  // for testing the alignment between Impress and Bullet colliders.
  //
  // Takes the transformation of the instance of btCollisionObject (e.g.
  // btRigidBody, btGhostObject) in subclass as the input.
  virtual CollisionShape GetCollisionShape(
      const btTransform& bt_trans) const = 0;

  virtual void ApplyScalingToBulletCollider() = 0;

  // Retrieves the node that owns this collidable shape.
  NodeHandle GetNode() const;

  // Retrieves the most up-to-date btTransform of the node associated with this
  // collidable shape
  btTransform GetNodeBtTransform();

#if IMP_RUNTIME(DEV)
  // Takes the transformation of the instance of btCollisionObject (e.g.
  // btRigidBody, btGhostObject) in subclass as the input.
  virtual void Visualize(const btTransform& bt_trans) const = 0;
#endif

 protected:
  // Since most Bullet collision shapes do not support non-uniform scaling, this
  // utility function can be used to compute the maximum scale of the three
  // dimensions and update the prev_scale arg if changed.
  static bool UpdatedEvenScale(float3& prev_scale, const float3& new_scale);

 protected:
  NodeHandle node_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_COLLIDABLE_SHAPES_COLLIDABLE_SHAPE_H_
