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

#include "core/physics/constraints/slider_constraint.h"

#include <memory>
#include <optional>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "bullet/src/LinearMath/btScalar.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/constraints/slider_constraint_state.proto.imp.h"
#include "core/physics/physics_helper.h"
#include "core/physics/rigid_body.h"
#include "core/view/utils/frame_time.h"

namespace imp {

void SliderConstraint::LoadDefaultStateValues() {
  state_.connected_pivot = kZero3;
  state_.connected_axis = kZAxis3f;
  state_.connected_up_axis = kYAxis3f;
  state_.pivot = kZero3;
  state_.axis = kZAxis3f;
  state_.up_axis = kYAxis3f;
  state_.auto_configure = true;
  state_.angular_lower_limit = 0.0f;
  state_.angular_upper_limit = 0.0f;
  state_.linear_lower_limit = 1.0f;
  state_.linear_upper_limit = -1.0f;
  state_.max_linear_motor_force = 0.0f;
  state_.max_linear_motor_velocity = 0.0f;
  state_.max_angular_motor_force = 0.0f;
  state_.max_angular_motor_velocity = 0.0f;

  state_.softness_linear_limit = SLIDER_CONSTRAINT_DEF_SOFTNESS;
  state_.restitution_linear_limit = SLIDER_CONSTRAINT_DEF_RESTITUTION;
  state_.damping_linear_limit = SLIDER_CONSTRAINT_DEF_DAMPING;

  state_.softness_angular_limit = SLIDER_CONSTRAINT_DEF_SOFTNESS;
  state_.restitution_angular_limit = SLIDER_CONSTRAINT_DEF_RESTITUTION;
  state_.damping_angular_limit = SLIDER_CONSTRAINT_DEF_DAMPING;

  state_.softness_ortho_linear_limit = SLIDER_CONSTRAINT_DEF_SOFTNESS;
  state_.restitution_ortho_linear_limit = SLIDER_CONSTRAINT_DEF_RESTITUTION;
  state_.damping_ortho_linear_limit = SLIDER_CONSTRAINT_DEF_DAMPING;

  state_.softness_ortho_angular_limit = SLIDER_CONSTRAINT_DEF_SOFTNESS;
  state_.restitution_ortho_angular_limit = SLIDER_CONSTRAINT_DEF_RESTITUTION;
  state_.damping_ortho_angular_limit = SLIDER_CONSTRAINT_DEF_DAMPING;
}

absl::Status SliderConstraint::Setup() {
  LoadDefaultStateValues();
  return SetupInternal();
}

absl::Status SliderConstraint::Setup(
    NodeHandle connected_node, bool auto_configure, float3 connected_pivot,
    float3 connected_axis, float3 connected_up_axis,
    std::optional<float3> pivot, std::optional<float3> axis,
    std::optional<float3> up_axis) {
  LoadDefaultStateValues();

  if (auto_configure && pivot.has_value()) {
    IMP_LOG(imp::WARNING) << "Pivot is ignored because auto_configure is true.";
  }

  state_.connected_node.AssignSceneHandleForNode(connected_node);
  state_.auto_configure = auto_configure;
  state_.connected_pivot = connected_pivot;
  state_.connected_axis = connected_axis;
  state_.connected_up_axis = connected_up_axis;
  if (pivot.has_value()) {
    state_.pivot = pivot.value();
  }
  if (axis.has_value()) {
    state_.axis = axis.value();
  }
  if (axis.has_value()) {
    state_.axis = axis.value();
  }
  if (up_axis.has_value()) {
    state_.up_axis = up_axis.value();
  }
  state_.up_axis = up_axis;
  return SetupInternal();
}

absl::Status SliderConstraint::SetupWithState() { return SetupInternal(); }

absl::Status SliderConstraint::SetupInternal() {
  state_.is_ready = true;
  if (InitializeWithNodes(state_.connected_node, GetNode()) !=
      absl::OkStatus()) {
    state_.is_ready = false;
  }

  if (!state_.is_ready) {
    bt_constraint_.reset();
    return absl::OkStatus();
  }

  if (state_.auto_configure) {
    state_.connected_pivot = ComputePivotAFromB(
        state_.connected_node, GetNode(), state_.pivot.value_or(kZero3));
    state_.connected_axis = ComputeAxisAFromB(state_.connected_node, GetNode(),
                                              state_.axis.value_or(kZAxis3f));
    state_.connected_up_axis = ComputeAxisAFromB(
        state_.connected_node, GetNode(), state_.up_axis.value_or(kYAxis3f));
  }

  btRigidBody* bt_rigid_body_A =
      const_cast<btRigidBody*>(&GetRigidBodyA()->GetBtRigidBody());
  btRigidBody* bt_rigid_body_B =
      GetRigidBodyB() == nullptr
          ? nullptr
          : const_cast<btRigidBody*>(&GetRigidBodyB()->GetBtRigidBody());

  if (bt_rigid_body_B == nullptr) {
    // then we will attach it to the world
    Transform<float> frame_in_A_t = Transform<float>(
        mat4::lookTo(state_.connected_axis, state_.connected_pivot,
                     state_.connected_up_axis));
    btTransform frame_in_A =
        ToBtTransform(frame_in_A_t.translation, frame_in_A_t.rotation);
    bt_constraint_ = std::make_unique<btSliderConstraint>(
        *bt_rigid_body_A, frame_in_A, /*useLinearReferenceFrameA*/ false);
  } else {
    float3 xAxis =
        normalize(cross(state_.connected_up_axis, state_.connected_axis));
    mat3f rot = mat3f(xAxis, state_.connected_up_axis, state_.connected_axis);
    btTransform frame_in_A =
        ToBtTransform(state_.connected_pivot, rot.toQuaternion());

    xAxis = normalize(cross(state_.up_axis.value_or(kYAxis3f),
                            state_.axis.value_or(kZAxis3f)));
    rot = mat3f(xAxis, state_.up_axis.value_or(kYAxis3f),
                state_.axis.value_or(kZAxis3f));
    btTransform frame_in_B =
        ToBtTransform(state_.pivot.value_or(kZero3), rot.toQuaternion());

    // useLinearReferenceFrameA is false, so the motor (if enabled), will try to
    // move body A in the direction of B's X-axis.
    bt_constraint_ = std::make_unique<btSliderConstraint>(
        *bt_rigid_body_A, *bt_rigid_body_B, frame_in_A, frame_in_B,
        /*useLinearReferenceFrameA*/ false);
  }

  SetLimitsFromState();
  SetMotorParametersFromState();
  SetSoftnessRestitutionDampingFromState();

  AddToPhysicsManager(true);

  return absl::OkStatus();
}

void SliderConstraint::SetAngularLimits(float lower_limit, float upper_limit) {
  state_.angular_lower_limit = lower_limit;
  state_.angular_upper_limit = upper_limit;

  bt_constraint_->setLowerAngLimit(ToRadians(state_.angular_lower_limit));
  bt_constraint_->setUpperAngLimit(ToRadians(state_.angular_upper_limit));
}

void SliderConstraint::SetLinearLimits(float lower_limit, float upper_limit) {
  state_.linear_lower_limit = lower_limit;
  state_.linear_upper_limit = upper_limit;

  bt_constraint_->setLowerLinLimit(state_.linear_lower_limit);
  bt_constraint_->setUpperLinLimit(state_.linear_upper_limit);
}

void SliderConstraint::SetLimitsFromState() {
  bt_constraint_->setLowerAngLimit(ToRadians(state_.angular_lower_limit));
  bt_constraint_->setUpperAngLimit(ToRadians(state_.angular_upper_limit));

  bt_constraint_->setLowerLinLimit(state_.linear_lower_limit);
  bt_constraint_->setUpperLinLimit(state_.linear_upper_limit);
}

void SliderConstraint::SetPoweredLinearMotor(float max_force,
                                             float max_velocity) {
  state_.max_linear_motor_force = max_force;
  state_.max_linear_motor_velocity = max_velocity;

  if (max_force == 0.0f && max_velocity == 0.0f) {
    bt_constraint_->setPoweredLinMotor(false);
  } else {
    bt_constraint_->setPoweredLinMotor(true);
    bt_constraint_->setMaxLinMotorForce(max_force);
    bt_constraint_->setTargetLinMotorVelocity(max_velocity);
  }
}

void SliderConstraint::SetPoweredAngularMotor(float max_force,
                                              float max_angular_velocity) {
  state_.max_angular_motor_force = max_force;
  state_.max_angular_motor_velocity = max_angular_velocity;

  if (max_force == 0.0f && max_angular_velocity == 0.0f) {
    bt_constraint_->setPoweredAngMotor(false);
  } else {
    bt_constraint_->setPoweredAngMotor(true);
    bt_constraint_->setMaxAngMotorForce(max_force);
    bt_constraint_->setTargetAngMotorVelocity(max_angular_velocity);
  }
}

void SliderConstraint::SetMotorParametersFromState() {
  if (state_.max_linear_motor_force == 0.0f &&
      state_.max_linear_motor_velocity == 0.0f) {
    bt_constraint_->setPoweredLinMotor(false);
  } else {
    bt_constraint_->setPoweredLinMotor(true);
    bt_constraint_->setMaxLinMotorForce(state_.max_linear_motor_force);
    bt_constraint_->setTargetLinMotorVelocity(state_.max_linear_motor_velocity);
  }
  if (state_.max_angular_motor_force == 0.0f &&
      state_.max_angular_motor_velocity == 0.0f) {
    bt_constraint_->setPoweredAngMotor(false);
  } else {
    bt_constraint_->setPoweredAngMotor(true);
    bt_constraint_->setMaxAngMotorForce(state_.max_angular_motor_force);
    bt_constraint_->setTargetAngMotorVelocity(
        state_.max_angular_motor_velocity);
  }
}

void SliderConstraint::SetSoftnessLinearLimit(float softness) {
  state_.softness_linear_limit = softness;
  bt_constraint_->setSoftnessLimLin(btScalar(state_.softness_linear_limit));
}

void SliderConstraint::SetRestitutionLinearLimit(float restitution) {
  state_.restitution_linear_limit = restitution;
  bt_constraint_->setRestitutionLimLin(
      btScalar(state_.restitution_linear_limit));
}

void SliderConstraint::SetDampingLinearLimit(float damping) {
  state_.damping_linear_limit = damping;
  bt_constraint_->setDampingLimLin(btScalar(state_.damping_linear_limit));
}

void SliderConstraint::SetSoftnessAngularLimit(float softness) {
  state_.softness_angular_limit = softness;
  bt_constraint_->setSoftnessLimAng(btScalar(state_.softness_angular_limit));
}

void SliderConstraint::SetRestitutionAngularLimit(float restitution) {
  state_.restitution_angular_limit = restitution;
  bt_constraint_->setRestitutionLimAng(
      btScalar(state_.restitution_angular_limit));
}

void SliderConstraint::SetDampingAngularLimit(float damping) {
  state_.damping_angular_limit = damping;
  bt_constraint_->setDampingLimAng(btScalar(state_.damping_angular_limit));
}

void SliderConstraint::SetSoftnessOrthogonalLinearLimit(float softness) {
  state_.softness_ortho_linear_limit = softness;
  bt_constraint_->setSoftnessOrthoLin(
      btScalar(state_.softness_ortho_linear_limit));
}

void SliderConstraint::SetRestitutionOrthogonalLinearLimit(float restitution) {
  state_.restitution_ortho_linear_limit = restitution;
  bt_constraint_->setRestitutionOrthoLin(
      btScalar(state_.restitution_ortho_linear_limit));
}

void SliderConstraint::SetDampingOrthogonalLinearLimit(float damping) {
  state_.damping_ortho_linear_limit = damping;
  bt_constraint_->setDampingOrthoLin(
      btScalar(state_.damping_ortho_linear_limit));
}

void SliderConstraint::SetSoftnessOrthogonalAngularLimit(float softness) {
  state_.softness_ortho_angular_limit = softness;
  bt_constraint_->setSoftnessOrthoAng(
      btScalar(state_.softness_ortho_angular_limit));
}

void SliderConstraint::SetRestitutionOrthogonalAngularLimit(float restitution) {
  state_.restitution_ortho_angular_limit = restitution;
  bt_constraint_->setRestitutionOrthoAng(
      btScalar(state_.restitution_ortho_angular_limit));
}

void SliderConstraint::SetDampingOrthogonalAngularLimit(float damping) {
  state_.damping_ortho_angular_limit = damping;
  bt_constraint_->setDampingOrthoAng(
      btScalar(state_.damping_ortho_angular_limit));
}

void SliderConstraint::SetSoftnessRestitutionDampingFromState() {
  bt_constraint_->setSoftnessLimLin(btScalar(state_.softness_linear_limit));
  bt_constraint_->setRestitutionLimLin(
      btScalar(state_.restitution_linear_limit));
  bt_constraint_->setDampingLimLin(btScalar(state_.damping_linear_limit));

  bt_constraint_->setSoftnessLimAng(btScalar(state_.softness_angular_limit));
  bt_constraint_->setRestitutionLimAng(
      btScalar(state_.restitution_angular_limit));
  bt_constraint_->setDampingLimAng(btScalar(state_.damping_angular_limit));

  bt_constraint_->setSoftnessOrthoLin(
      btScalar(state_.softness_ortho_linear_limit));
  bt_constraint_->setRestitutionOrthoLin(
      btScalar(state_.restitution_ortho_linear_limit));
  bt_constraint_->setDampingOrthoLin(
      btScalar(state_.damping_ortho_linear_limit));

  bt_constraint_->setSoftnessOrthoAng(
      btScalar(state_.softness_ortho_angular_limit));
  bt_constraint_->setRestitutionOrthoAng(
      btScalar(state_.restitution_ortho_angular_limit));
  bt_constraint_->setDampingOrthoAng(
      btScalar(state_.damping_ortho_angular_limit));
}

void SliderConstraint::Update(const FrameTime& frame_time) {
  CheckIntegrityAndUpdate();
}

void SliderConstraint::OnRigidBodiesChanged() {
  // delete the constraint and create it again
  AddToPhysicsManager(false);
  bt_constraint_.reset();

  absl::Status status = SetupInternal();
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to setup Point2PointConstraint: " << status;
  }
}

void SliderConstraint::OnIsfStateChanged() {
  // TODO: (broken link) - Don't recreate the constraint if only the properties
  // were changed. Use bullet's specific functions to update the settings.
  // Only recreate if the nodes are changed.
  AddToPhysicsManager(false);
  bt_constraint_.reset();
  absl::Status status = SetupInternal();
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to setup Point2PointConstraint: " << status;
  }
}

}  // namespace imp
