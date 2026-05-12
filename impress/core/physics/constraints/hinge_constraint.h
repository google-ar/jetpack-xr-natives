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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_HINGE_CONSTRAINT_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_HINGE_CONSTRAINT_H_

#include <memory>

#include "absl/status/status.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_id.h"
#include "core/ncsb/update_phase.h"
#include "core/physics/constraints/base_constraint.h"
#include "core/physics/constraints/hinge_constraint_state.proto.imp.h"
#include "core/physics/physics_manager.h"
#include "core/physics/rigid_body.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Implements a hinge constraint: a joint that allows rotation around a single
// axis. It can be between two rigid bodies or between a rigidbody and the
// world. A common example would be a door hinged to a door frame.
class HingeConstraint : public BaseConstraint, public Component {
 public:
  // Update before physics manager to ensure that the constraint is up to date
  // with rigid body states (maybe a rigid body was removed and triggered a
  // constraint removal). We want those changes (removal from physics world) to
  // take effect in the current frame simulation.
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
  // 1. Constructs a hinge constraint between two rigid bodies if the owner
  // node of this component has a RigidBody component.
  //
  // (or) 2. Constructs a world-locked hinge constraint if the owner node hold
  // of this component doesn't have a RigidBody component.
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
  //     axis:
  //          - the axis of the constraint in the owner node's local space.
  //
  //    auto_configure:
  //          - If true, set that constraint (pivot and axis) to respect the
  //          current transformation of node(s).
  //          - If false, set the constraint to respect the value of
  //          `connected_pivot` and `connected_axis`, which may move the nodes.
  //
  //     connected_pivot:
  //          - the pivot of the constraint frame in the connected node's local
  //          space.
  //
  //     connected_axis:
  //          - the axis of the constraint in the connected node's local space.
  //
  // Note: that the axes pairs specified (connected_axis and connected_up_axis,
  // axis and up_axis) must be orthogonal.
  absl::Status Setup(NodeHandle connected_node, float3 pivot = kZero3,
                     float3 axis = kZAxis3f, bool auto_configure = true,
                     float3 connected_pivot = kZero3,
                     float3 connected_axis = kZAxis3f);

  // Sets the constraint with the already filled state.
  absl::Status SetupWithState();

  // Setup the constraint with the default values (identity transforms)
  // The axis is local Z
  absl::Status Setup();

  // Cleanup the constraint.
  void Cleanup() { AddToPhysicsManager(false); }

  // Called when the constraint is activated or deactivated.
  void OnActiveStatusChanged(bool is_active) { AddToPhysicsManager(is_active); }

  // Check the costraint's integrity (rigidbodies active states)
  void Update(const FrameTime& frame_time);

  void OnIsfStateChanged();

  bool IsReady() const { return state_.is_ready; }

  // The limits in degrees of the hinge. Zero position represents the initial
  // position of the bodies.
  void SetLimits(float lower_limit, float upper_limit);

  // Error correction speed: how quickly the constraint tries to correct
  // positional errors after the limits are reached.
  void SetBiasFactor(float bias_factor);

  // Damping factor: how quickly the slowing down is happening when the hinge is
  // close to the limit.
  void SetRelaxationFactor(float relaxation_factor);

  // enables the motor: it applies a torque to the hinge, which rotates the
  // hinge around the axis.
  // velocity is the maximum angular velocity (radians per second)
  // max_impulse is the maximum torque impulse per timestep.
  void SetAngularMotor(bool enabled, float velocity, float max_impulse);

  // Get the limits of the hinge in degrees.
  float GetLowerLimit() const { return state_.lower_limit; }
  float GetUpperLimit() const { return state_.upper_limit; }

  float GetHingeAngle() const;

 protected:
  void OnRigidBodiesChanged() override;

 private:
  btTypedConstraint* GetBtConstraint() const override {
    return bt_constraint_.get();
  }

  absl::Status SetupInternal();

  void SetLimitsFromState();
  void SetMotorParametersFromState();

  void LoadDefaultStateValues();

#if IMP_RUNTIME(DEV)
  void Visualize();
  mat4f GetFrameB(bool world_space = false);

  float hinge_start_angle_;
#endif

  HingeConstraintState state_;
  std::unique_ptr<btHingeConstraint> bt_constraint_;

 public:
  using IsfInfo = IsfInfo<&HingeConstraint::state_, IsfDependencies<RigidBody>>;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_HINGE_CONSTRAINT_H_
