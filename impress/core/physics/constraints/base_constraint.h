/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_BASE_CONSTRAINT_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_BASE_CONSTRAINT_H_

#include "absl/status/status.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/physics_manager.h"
#include "core/physics/rigid_body.h"

namespace imp {

// Base class for all constraints.
// A constraint connects two rigid bodies and enforce a set of limits for the
// movement (translation and rotation) of these rigid bodies. Real life analogy
// can be a hinge, a slider or a ball joint. This class contains some common
// methods for constraint management and configuration. The constraint has its
// own local space (coordinate system), called "frame". To setup the constraint,
// it requires the transformation from the "frame" to the local space of each
// rigid body.
// The notations A and B are used to refer to the rigid bodies. A is the rigid
// body that is being constrained and B is the rigid body that is attached to
// the node that has the constraint component. B can be null, meaning that A is
// constrained to the world.
class BaseConstraint {
 public:
  virtual ~BaseConstraint() = default;

 protected:
  absl::Status InitializeWithNodes(NodeHandle connected_node,
                                   NodeHandle owner_node);

  // Returns the rigid body A or B (constrained node or current node's)
  RigidBody* GetRigidBodyA();
  RigidBody* GetRigidBodyB();

  void AddToPhysicsManager(bool add);

  virtual btTypedConstraint* GetBtConstraint() const = 0;

  void CheckIntegrityAndUpdate();

  virtual void OnRigidBodiesChanged() {}

  // Computes the pivot of the constraint in A's local space.
  static float3 ComputePivotAFromB(NodeHandle node_a, NodeHandle node_b,
                                   const float3& pivot_in_b);
  // Computes an axis of the constraint in A's local space.
  static float3 ComputeAxisAFromB(NodeHandle node_a, NodeHandle node_b,
                                  const float3& axis_in_b);

  bool IsActiveInWorld() const;

 private:
  bool added_to_physics_manager_ = false;

  NodeHandle owner_node_;
  NodeHandle connected_node_;

  bool owner_node_valid_prev_ = false;
  bool connected_node_valid_prev_ = false;

  PhysicsManager* physics_manager_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_BASE_CONSTRAINT_H_
