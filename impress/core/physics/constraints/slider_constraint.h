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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_SLIDER_CONSTRAINT_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_SLIDER_CONSTRAINT_H_

#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_id.h"
#include "core/ncsb/update_phase.h"
#include "core/physics/constraints/base_constraint.h"
#include "core/physics/constraints/slider_constraint_state.proto.imp.h"
#include "core/physics/physics_manager.h"
#include "core/physics/rigid_body.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Implements a slider constraint.
// The bodies can slide along a common axis. Example: a drawer sliding in or
// out.
class SliderConstraint : public BaseConstraint, public Component {
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

  absl::Status Setup();

  // Specify the connected node and whether to auto-configure the pivots and
  // axis. It will perform one of the following:
  //
  // 1. Constructs a slider constraint between two rigid bodies if the owner
  // node of this component has a RigidBody component.
  //
  // (or) 2. Constructs a world-locked slider constraint if the owner node hold
  // of this component doesn't have a RigidBody component.
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
  //          - the axis of the slider in the owner node's local space.
  //
  //     up_axis:
  //          - an axis that is orthogonal to the slider axis and is used to
  //          define the rotation (orientation) of the slider in the local 3D
  //          space of the owner node.
  //
  //     auto_configure:
  //          - If true, set that constraint (pivot and axis) to respect the
  //          current transformation of node(s).
  //          - If false, set the constraint to respect the value of
  //          `connected_pivot`, `connected_axis` and `connected_up_axis` which
  //          may move the nodes.
  //
  //     connected_pivot:
  //          - the pivot of the constraint frame in the connected node(that
  //          holds rigid body A)'s local space.
  //
  //     connected_axis:
  //          - the axis of the slider in the connected node(that holds rigid
  //          body A)'s local space.
  //
  //     connected_up_axis:
  //          - an axis that is orthogonal to the slider axis and is used to
  //          define the rotation (orientation) of the slider in the local 3D
  //          space of the connected node.
  //
  // Note: the axes pairs specified (connected_axis and connected_up_axis, axis
  // and up_axis) must be orthogonal.
  absl::Status Setup(NodeHandle connected_node, float3 pivot = kZero3,
                     float3 axis = kZAxis3f, float3 up_axis = kYAxis3f,
                     bool auto_configure = true,
                     float3 connected_pivot = kZero3,
                     float3 connected_axis = kZAxis3f,
                     float3 connected_up_axis = kYAxis3f);

  // Sets the constraint with the already filled state.
  absl::Status SetupWithState();

  // Cleanup the constraint.
  void Cleanup() { AddToPhysicsManager(false); }

  // Called when the constraint is activated or deactivated.
  void OnActiveStatusChanged(bool is_active) { AddToPhysicsManager(is_active); }

  // Check the costraint's integrity (rigidbodies active states)
  void Update(const FrameTime& frame_time);

  void OnIsfStateChanged();

  bool IsReady() const { return state_.is_ready; }

  // Limits for the movement
  void SetLinearLimits(float lower_limit, float upper_limit);

  // Enables the motor with desired force or velocity to be reached
  // Use zero values to disable the motor.
  // Since useLinearReferenceFrameA is true, the motor will try to move body B
  // in the direction of A's X-axis.
  void SetPoweredLinearMotor(float max_force, float max_velocity);

  // Softness when hitting linear limit
  // softness: 0 to 1, 1 means a very soft limit, like a spring
  void SetSoftnessLinearLimit(float softness);
  // Bounciness when hitting linear limits
  // restitution: 0 to 1, how much bounce is happening when hitting the limits
  void SetRestitutionLinearLimit(float restitution);
  // Damping when hitting linear limits
  // damping: 0 to 1, it means how much velocity (or energy) is lost when
  // approaching/hitting the limits.
  void SetDampingLinearLimit(float damping);

  // Softness when hitting the limits on the orthogonal axes (against the
  // constraint axis) softness: 0 to 1, 1 means a very soft limit, like a spring
  void SetSoftnessOrthogonalLinearLimit(float softness);
  // Bounciness when hitting the limits on the orthogonal axes (against the
  // constraint axis) restitution: 0 to 1, how much bounce is happening when
  // hitting the limits
  void SetRestitutionOrthogonalLinearLimit(float restitution);
  // Damping when hitting the limits on the orthogonal axes
  // damping: 0 to 1, it means how much velocity (or energy) is lost when
  // approaching/hitting the limits.
  void SetDampingOrthogonalLinearLimit(float damping);

 protected:
  void SetLimitsFromState();

  void SetMotorParametersFromState();

  void SetSoftnessRestitutionDampingFromState();

  btTypedConstraint* GetBtConstraint() const override {
    return bt_constraint_.get();
  }

  void OnRigidBodiesChanged() override;

 private:
  absl::Status SetupInternal();
  void LoadDefaultStateValues();

  SliderConstraintState state_;
  std::unique_ptr<btSliderConstraint> bt_constraint_;

 public:
  using IsfInfo =
      IsfInfo<&SliderConstraint::state_, IsfDependencies<RigidBody>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_CONSTRAINTS_SLIDER_CONSTRAINT_H_
