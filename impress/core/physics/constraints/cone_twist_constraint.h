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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_CONE_TWIST_CONSTRAINT_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_CONE_TWIST_CONSTRAINT_H_

#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_id.h"
#include "core/ncsb/update_phase.h"
#include "core/physics/constraints/base_constraint.h"
#include "core/physics/constraints/cone_twist_constraint_state.proto.imp.h"
#include "core/physics/physics_manager.h"
#include "core/physics/rigid_body.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// A cone twist constraint allows rotation around all three axes, but
// with limits (hence the cone naming). It has a behavior similar to a
// ball-and-socket joint, like the shoulder or hips from a human body.
class ConeTwistConstraint : public BaseConstraint, public Component {
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

  // Specify the connected node and whether to auto-configure the pivots and
  // axis. It will perform one of the following:
  //
  // 1. Constructs a cone twist constraint between two rigid bodies if the
  // owner node of this component has a RigidBody component.
  //
  // or 2. Constructs a world-locked cone twist constraint if the owner node
  // hold of this component doesn't have a RigidBody component.
  //
  // Parameters:
  //     connected_node:
  //          - another node that holds one or the only rigid body of this
  //          constraint.
  //
  //     pivot:
  //          - the pivot of the constraint frame in the owner node's local
  //          space.
  //
  //     axis:
  //          - the axis of the constraint in the owner node's local space.
  //
  //     up_axis:
  //          - an axis that is orthogonal to the constraint axis and is used to
  //          define the rotation (orientation) of the constraint in the local
  //          3D space of the owner node.
  //
  //     auto_configure:
  //          - If true, set that constraint (pivot and axis) to respect
  //          the current transformation of node(s).
  //          - If false, set the constraint to respect the value of
  //          `connected_pivot`, `connected_axis` and `connected_up_axis` which
  //          may move the nodes.
  //
  //     connected_pivot:
  //          - the pivot of the constraint frame in the connected node(that
  //          holds rigid body A)'s local space.
  //
  //     connected_axis:
  //          - the axis of the constraint in the connected node(that holds
  //          rigid body A)'s local space.
  //
  //     connected_up_axis:
  //          - an axis that is orthogonal to the constraint axis and is used to
  //          define the rotation (orientation) of the constraint in the local
  //          3D space of the connected node.
  //
  // Note: that the axes pairs specified (connected_axis and connected_up_axis,
  // axis and up_axis) must be orthogonal.
  absl::Status Setup(NodeHandle connected_node, float3 pivot = kZero3,
                     float3 axis = kZAxis3f, float3 up_axis = kYAxis3f,
                     bool auto_configure = true,
                     float3 connected_pivot = kZero3,
                     float3 connected_axis = kZAxis3f,
                     float3 connected_up_axis = kYAxis3f);

  // Sets the constraint with the already filled state.
  absl::Status SetupWithState();

  absl::Status Setup();

  // Cleanup the constraint.
  void Cleanup() { AddToPhysicsManager(false); }

  // Called when the constraint is activated or deactivated.
  void OnActiveStatusChanged(bool is_active) { AddToPhysicsManager(is_active); }

  // Check the costraint's integrity (rigidbodies active states)
  void Update(const FrameTime& frame_time);

  void OnIsfStateChanged();

  bool IsReady() const { return state_.is_ready; }

  // Sets the limits on the swing and twist.
  // Swings: Limit how far the joint can move away from the twist axis (Y/Z
  // axes) Twist: Limit how far the joint can move around the main axis (X axis)
  void SetLimits(float swingSpanOne, float swingSpanTwo, float twistSpan);

  // Fine tuning of the constraint
  // Softness: controls how "soft" the limits are - how much correction is
  // applied when reaching the limits. 0 to 1: zero means completely soft, 1
  // means completely hard (full correction when hitting the limits)
  void SetSoftness(float softness);

  // Bias factor: how fast the joint tries to correct when the limit is hit
  void SetBiasFactor(float bias_factor);

  // Relaxation factor: how much damping is applied when the limit is hit
  void SetRelaxationFactor(float relaxation_factor);

  // Enable the motor driving the joint
  void SetMotorEnabled(bool motor_enabled);

  // Set the maximum motor impulse (torque x timestep)
  void SetMaxMotorImpulse(float max_motor_impulse);

  // Set the target orientation to which the motor will drive the joint.
  void SetMotorTarget(quatf target_orientation);

  // Is the motor enabled?
  bool IsMotorEnabled() const;

  // Get the maximum motor impulse
  float GetMaxMotorImpulse() const;

  // Get the motor target orientation
  quatf GetMotorTarget() const;

  // Is the constraint past the swing limit?
  bool IsPastSwingLimit() const;

 protected:
  ConeTwistConstraintState state_;

  btTypedConstraint* GetBtConstraint() const override {
    return bt_constraint_.get();
  }

  void OnRigidBodiesChanged() override;

 private:
  absl::Status SetupInternal();
  void SetLimitsFromState();
  void SetMotorParametersFromState();
  void LoadDefaultStateValues();

  std::unique_ptr<btConeTwistConstraint> bt_constraint_;

 public:
  using IsfInfo =
      IsfInfo<&ConeTwistConstraint::state_, IsfDependencies<RigidBody>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_GENERIC_6DOF_CONSTRAINT_H_
