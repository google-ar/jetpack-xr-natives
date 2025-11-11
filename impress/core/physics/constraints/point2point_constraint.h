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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_POINT2POINT_CONSTRAINT_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_POINT2POINT_CONSTRAINT_H_

#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_id.h"
#include "core/ncsb/update_phase.h"
#include "core/physics/constraints/base_constraint.h"
#include "core/physics/constraints/point2point_constraint_state.proto.imp.h"
#include "core/physics/physics_manager.h"
#include "core/physics/rigid_body.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Implements a point-to-point constraint: a ball-and-socket type of constraint.
// It locks two pivot points together in space, allowing free rotation, but no
// separation (translation is locked)
class Point2PointConstraint : public BaseConstraint, public Component {
 public:
  // Update before physics manager to ensure that the constraint is up to date
  // with rigid body states.
  static constexpr UpdatePhase kUpdatePhase = PhysicsManager::kUpdatePhase;
  static constexpr Component::UpdateMode kUpdateMode =
      Component::UpdateMode::kAlwaysUpdate;
  using UpdateDependents = UpdateIds<PhysicsManager>;
  // Cleanup before rigid bodies to be sure that there are no constraints
  // depending on rigid bodies.
  using CleanupDependents = CleanupIds<RigidBody>;

  // Specify the connected node and whether to auto-configure the pivots. It
  // will perform one of the following:
  //
  // 1. Construct a point-to-point constraint between two rigid body if the
  // owner node of this component has a RigidBody component.
  //
  // (or) 2. Construct a world-locked point-to-point constraint if the owner
  // node hold of this component doesn't have a RigidBody component.
  //
  // Parameters:
  //    connected_node:
  //          - another node that holds one or the only rigid body of this
  //          constraint.
  //
  //     pivot:
  //          - the pivot of the constraint frame in the owner node's local
  //          space.
  //
  //    auto_configure:
  //          - If true, set that constraint to respect the current
  //          transformation of node(s).
  //          - If false, set the constraint to respect the value of
  //          `connected_pivot`, which may move the nodes.
  //
  //     connected_pivot:
  //          - the pivot of the constraint frame in the connected node's local
  //          space.
  absl::Status Setup(NodeHandle connected_node, float3 pivot = kZero3,
                     bool auto_configure = true,
                     float3 connected_pivot = kZero3);

  // Sets the constraint with the already filled state.
  absl::Status SetupWithState();

  absl::Status Setup();

  // Cleanup the constraint.
  void Cleanup();

  // Called when the constraint is activated or deactivated.
  void OnActiveStatusChanged(bool is_active) { AddToPhysicsManager(is_active); }

  // Check the costraint's integrity (rigidbodies active states)
  void Update(const FrameTime& frame_time);

  void OnIsfStateChanged();

  bool IsReady() const { return state_.is_ready; }

 protected:
  void OnRigidBodiesChanged() override;

 private:
  btTypedConstraint* GetBtConstraint() const override {
    return bt_constraint_.get();
  }
  absl::Status SetupInternal();

  Point2PointConstraintState state_;
  std::unique_ptr<btPoint2PointConstraint> bt_constraint_;

 public:
  using IsfInfo =
      IsfInfo<&Point2PointConstraint::state_, IsfDependencies<RigidBody>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_POINT2POINT_CONSTRAINT_H_
