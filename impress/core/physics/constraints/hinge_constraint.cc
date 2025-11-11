// Copyright 2025 Google LLC
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

#include "core/physics/constraints/hinge_constraint.h"

#include <memory>
#include <optional>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "bullet/src/LinearMath/btScalar.h"
#include "bullet/src/LinearMath/btVector3.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/physics_helper.h"
#include "core/physics/rigid_body.h"
#include "core/view/utils/frame_time.h"

namespace imp {

void HingeConstraint::LoadDefaultStateValues() {
  // low greater then high means no limits/free movement around the axis.
  state_.pivot = kZero3;
  state_.axis = kZAxis3f;
  state_.connected_pivot = kZero3;
  state_.connected_axis = kZAxis3f;
  state_.auto_configure = true;
  state_.lower_limit = 1.0f;
  state_.upper_limit = -1.0f;
  state_.bias_factor = 0.3f;
  state_.relaxation_factor = 1.0f;
  state_.angular_motor_enabled = false;
  state_.angular_motor_velocity = 0.0f;
  state_.angular_motor_max_impulse = -1.0f;
}

absl::Status HingeConstraint::Setup() {
  LoadDefaultStateValues();
  return SetupInternal();
}

absl::Status HingeConstraint::Setup(NodeHandle connected_node, float3 pivot,
                                    float3 axis, bool auto_configure,
                                    float3 connected_pivot,
                                    float3 connected_axis) {
  LoadDefaultStateValues();

  state_.connected_node.AssignSceneHandleForNode(connected_node);
  state_.auto_configure = auto_configure;
  state_.pivot = pivot;
  state_.axis = axis;
  state_.connected_pivot = connected_pivot;
  state_.connected_axis = connected_axis;

  return SetupInternal();
}

absl::Status HingeConstraint::SetupWithState() { return SetupInternal(); }

absl::Status HingeConstraint::SetupInternal() {
  state_.is_ready = true;
  if (InitializeWithNodes(state_.connected_node, GetNode()) !=
      absl::OkStatus()) {
    state_.is_ready = false;
  }

  if (!state_.is_ready) {
    bt_constraint_.reset();
    return absl::OkStatus();
  }

  float3 axis = state_.axis.value_or(kZAxis3f);

  float3 connected_pivot;
  float3 connected_axis;

  btRigidBody* bt_rigid_body_A =
      const_cast<btRigidBody*>(&GetRigidBodyA()->GetBtRigidBody());
  btRigidBody* bt_rigid_body_B =
      GetRigidBodyB() == nullptr
          ? nullptr
          : const_cast<btRigidBody*>(&GetRigidBodyB()->GetBtRigidBody());

  if (state_.auto_configure || bt_rigid_body_B == nullptr) {
    connected_pivot =
        ComputePivotAFromB(state_.connected_node, GetNode(), state_.pivot);
    connected_axis = ComputeAxisAFromB(state_.connected_node, GetNode(), axis);
  } else {
    connected_pivot = state_.connected_pivot.value_or(kZero3);
    connected_axis = state_.connected_axis.value_or(kZAxis3f);
  }

  btVector3 bt_pivot_in_A = ToBtVector3(connected_pivot);
  btVector3 bt_axis_in_A = ToBtVector3(connected_axis);

  if (bt_rigid_body_B == nullptr) {
    // then we will attach it to the world
    bt_constraint_ = std::make_unique<btHingeConstraint>(
        *bt_rigid_body_A, bt_pivot_in_A, bt_axis_in_A);
  } else {
    btVector3 bt_pivot_in_B = ToBtVector3(state_.pivot);
    btVector3 bt_axis_in_B = ToBtVector3(axis);

    bt_constraint_ = std::make_unique<btHingeConstraint>(
        *bt_rigid_body_A, *bt_rigid_body_B, bt_pivot_in_A, bt_pivot_in_B,
        bt_axis_in_A, bt_axis_in_B);
  }

  state_.axis = axis;
  state_.connected_pivot = connected_pivot;
  state_.connected_axis = connected_axis;

  SetLimitsFromState();
  SetMotorParametersFromState();

  AddToPhysicsManager(true);

  return absl::OkStatus();
}

void HingeConstraint::SetLimits(float lower_limit, float upper_limit) {
  state_.lower_limit = lower_limit;
  state_.upper_limit = upper_limit;
  SetLimitsFromState();
}

void HingeConstraint::SetLimitsFromState() {
  bt_constraint_->setLimit(ToRadians(state_.lower_limit),
                           ToRadians(state_.upper_limit),
                           /*softness is not used*/ 0.0f, state_.bias_factor,
                           state_.relaxation_factor);
}

void HingeConstraint::SetBiasFactor(float bias_factor) {
  state_.bias_factor = bias_factor;
  SetLimitsFromState();
}

void HingeConstraint::SetRelaxationFactor(float relaxation_factor) {
  state_.relaxation_factor = relaxation_factor;
  SetLimitsFromState();
}

void HingeConstraint::SetAngularMotor(bool enabled, float velocity,
                                      float max_impulse) {
  state_.angular_motor_enabled = enabled;
  state_.angular_motor_velocity = velocity;
  state_.angular_motor_max_impulse = max_impulse;
  SetMotorParametersFromState();
}

void HingeConstraint::SetMotorParametersFromState() {
  bt_constraint_->enableAngularMotor(
      state_.angular_motor_enabled, btScalar(state_.angular_motor_velocity),
      btScalar(state_.angular_motor_max_impulse));
}

float HingeConstraint::GetHingeAngle() const {
  return ToDegrees(bt_constraint_->getHingeAngle());
}

void HingeConstraint::Update(const FrameTime& frame_time) {
  CheckIntegrityAndUpdate();
}

void HingeConstraint::OnRigidBodiesChanged() {
  // delete the constraint and create it again
  AddToPhysicsManager(false);
  bt_constraint_.reset();

  absl::Status status = SetupInternal();
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to setup HingeConstraint: " << status;
  }
}

void HingeConstraint::OnIsfStateChanged() {
  // TODO: (broken link) - Don't recreate the constraint if only the properties
  // were changed. Use bullet's specific functions to update the settings.
  // Only recreate if the nodes are changed.
  AddToPhysicsManager(false);
  bt_constraint_.reset();
  absl::Status status = SetupInternal();
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to setup HingeConstraint: " << status;
  }
}

}  // namespace imp
