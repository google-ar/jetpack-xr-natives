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

#include "core/physics/constraints/cone_twist_constraint.h"

#include <memory>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "bullet/src/LinearMath/btScalar.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/physics_helper.h"
#include "core/view/utils/frame_time.h"

namespace imp {

void ConeTwistConstraint::LoadDefaultStateValues() {
  state_.pivot = kZero3;
  state_.axis = kZAxis3f;
  state_.up_axis = kYAxis3f;
  state_.connected_pivot = kZero3;
  state_.connected_axis = kZAxis3f;
  state_.connected_up_axis = kYAxis3f;
  state_.swing_span_one = BT_LARGE_FLOAT;
  state_.swing_span_two = BT_LARGE_FLOAT;
  state_.twist_span = BT_LARGE_FLOAT;
  state_.softness = 1.0f;
  state_.bias_factor = 0.3f;
  state_.relaxation_factor = 1.0f;
  state_.motor_enabled = false;
  state_.max_motor_impulse = -1;
  state_.motor_target = kIdentityQuatf;
  state_.auto_configure = true;
}

absl::Status ConeTwistConstraint::Setup() {
  LoadDefaultStateValues();
  return SetupInternal();
}

absl::Status ConeTwistConstraint::Setup(NodeHandle connected_node, float3 pivot,
                                        float3 axis, float3 up_axis,
                                        bool auto_configure,
                                        float3 connected_pivot,
                                        float3 connected_axis,
                                        float3 connected_up_axis) {
  LoadDefaultStateValues();

  state_.connected_node.AssignSceneHandleForNode(connected_node);
  state_.auto_configure = auto_configure;

  state_.pivot = pivot;
  state_.axis = axis;
  state_.up_axis = up_axis;
  state_.connected_pivot = connected_pivot;
  state_.connected_axis = connected_axis;
  state_.connected_up_axis = connected_up_axis;
  return SetupInternal();
}

absl::Status ConeTwistConstraint::SetupWithState() { return SetupInternal(); }

absl::Status ConeTwistConstraint::SetupInternal() {
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
  float3 up_axis = state_.up_axis.value_or(kYAxis3f);

  float3 connected_pivot;
  float3 connected_axis;
  float3 connected_up_axis;

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
    connected_up_axis =
        ComputeAxisAFromB(state_.connected_node, GetNode(), up_axis);
  } else {
    connected_pivot = state_.connected_pivot.value_or(kZero3);
    connected_axis = state_.connected_axis.value_or(kZAxis3f);
    connected_up_axis = state_.connected_up_axis.value_or(kYAxis3f);
  }

  float3 xAxis = normalize(cross(connected_up_axis, connected_axis));
  mat3f rot = mat3f(xAxis, connected_up_axis, connected_axis);
  btTransform frame_in_A = ToBtTransform(connected_pivot, rot.toQuaternion());

  if (bt_rigid_body_B == nullptr) {
    // btConeTwistConstraint implementation moves the pivot to origin.
    // So, to have a consistent behavior with the other constraints, we do the
    // same implementation here: we use the fixed body as the world anchor, and
    // we follow the two-body implementation.
    xAxis = normalize(cross(up_axis, axis));
    rot = mat3f(xAxis, up_axis, axis);
    Transform<float> t =
        Transform<float>(GetNode()->GetWorldTrs() * mat4f(rot, state_.pivot));

    btTransform frame_in_B = ToBtTransform(t.translation, t.rotation);

    bt_constraint_ = std::make_unique<btConeTwistConstraint>(
        *bt_rigid_body_A, btConeTwistConstraint::getFixedBody(), frame_in_A,
        frame_in_B);
  } else {
    xAxis = normalize(cross(up_axis, axis));
    rot = mat3f(xAxis, up_axis, axis);
    btTransform frame_in_B = ToBtTransform(state_.pivot, rot.toQuaternion());

    bt_constraint_ = std::make_unique<btConeTwistConstraint>(
        *bt_rigid_body_A, *bt_rigid_body_B, frame_in_A, frame_in_B);
  }

  state_.axis = axis;
  state_.up_axis = up_axis;
  state_.connected_pivot = connected_pivot;
  state_.connected_axis = connected_axis;
  state_.connected_up_axis = connected_up_axis;

  SetLimitsFromState();
  SetMotorParametersFromState();

  AddToPhysicsManager(true);

  return absl::OkStatus();
}

void ConeTwistConstraint::SetLimits(float swingSpanOne, float swingSpanTwo,
                                    float twistSpan) {
  state_.swing_span_one = swingSpanOne;
  state_.swing_span_two = swingSpanTwo;
  state_.twist_span = twistSpan;

  SetLimitsFromState();
}

void ConeTwistConstraint::SetSoftness(float softness) {
  state_.softness = softness;
  SetLimitsFromState();
}

void ConeTwistConstraint::SetBiasFactor(float bias_factor) {
  state_.bias_factor = bias_factor;
  SetLimitsFromState();
}

void ConeTwistConstraint::SetRelaxationFactor(float relaxation_factor) {
  SetLimitsFromState();
}

void ConeTwistConstraint::SetLimitsFromState() {
  if (state_.relaxation_factor == 0.0) {
    IMP_LOG(imp::WARNING) << "ConeTwistConstraint: Relaxation factor should not be 0.0";
    state_.relaxation_factor = 0.01f;
  }
  bt_constraint_->setLimit(ToRadians(state_.swing_span_one),
                           ToRadians(state_.swing_span_two),
                           ToRadians(state_.twist_span), state_.softness,
                           state_.bias_factor, state_.relaxation_factor);
}

void ConeTwistConstraint::SetMotorEnabled(bool motor_enabled) {
  state_.motor_enabled = motor_enabled;
  bt_constraint_->enableMotor(motor_enabled);
}

void ConeTwistConstraint::SetMaxMotorImpulse(float max_motor_impulse) {
  state_.max_motor_impulse = max_motor_impulse;
  bt_constraint_->setMaxMotorImpulse(state_.max_motor_impulse);
}

void ConeTwistConstraint::SetMotorTarget(quatf target_orientation) {
  state_.motor_target = target_orientation;
  bt_constraint_->setMotorTarget(ToBtQuaternion(target_orientation));
}

void ConeTwistConstraint::SetMotorParametersFromState() {
  bt_constraint_->enableMotor(state_.motor_enabled);
  if (state_.motor_enabled) {
    bt_constraint_->setMaxMotorImpulse(state_.max_motor_impulse);
    bt_constraint_->setMotorTarget(ToBtQuaternion(state_.motor_target));
  }
}

bool ConeTwistConstraint::IsMotorEnabled() const {
  return bt_constraint_->isMotorEnabled();
}

float ConeTwistConstraint::GetMaxMotorImpulse() const {
  return bt_constraint_->getMaxMotorImpulse();
}

quatf ConeTwistConstraint::GetMotorTarget() const {
  return ToQuaternion(bt_constraint_->getMotorTarget());
}

bool ConeTwistConstraint::IsPastSwingLimit() const {
  return bt_constraint_->isPastSwingLimit();
}

void ConeTwistConstraint::Update(const FrameTime& frame_time) {
  CheckIntegrityAndUpdate();
}

void ConeTwistConstraint::OnRigidBodiesChanged() {
  // delete the constraint and create it again
  AddToPhysicsManager(false);
  bt_constraint_.reset();

  absl::Status status = SetupInternal();
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to setup Point2PointConstraint: " << status;
  }
}

void ConeTwistConstraint::OnIsfStateChanged() {
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
