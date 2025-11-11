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

#include "core/physics/constraints/generic_6dof_constraint.h"

#include <memory>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "bullet/src/LinearMath/btVector3.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/physics_helper.h"
#include "core/physics/rigid_body.h"
#include "core/view/utils/frame_time.h"

namespace imp {

void Generic6DofConstraint::LoadDefaultStateValues() {
  state_.pivot = kZero3;
  state_.axis = kZAxis3f;
  state_.up_axis = kYAxis3f;
  state_.connected_pivot = kZero3;
  state_.connected_axis = kZAxis3f;
  state_.connected_up_axis = kYAxis3f;
  state_.auto_configure = true;
  state_.angular_lower_limit = float3(0.0f, 0.0f, 0.0f);
  state_.angular_upper_limit = float3(0.0f, 0.0f, 0.0f);
  state_.linear_lower_limit = float3(0.0f, 0.0f, 0.0f);
  state_.linear_upper_limit = float3(0.0f, 0.0f, 0.0f);
}

absl::Status Generic6DofConstraint::Setup() {
  LoadDefaultStateValues();
  return SetupInternal();
}

absl::Status Generic6DofConstraint::Setup(NodeHandle connected_node,
                                          float3 pivot, float3 axis,
                                          float3 up_axis, bool auto_configure,
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

absl::Status Generic6DofConstraint::SetupWithState() { return SetupInternal(); }

absl::Status Generic6DofConstraint::SetupInternal() {
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
    state_.connected_pivot =
        ComputePivotAFromB(state_.connected_node, GetNode(), state_.pivot);
    state_.connected_axis = ComputeAxisAFromB(state_.connected_node, GetNode(),
                                              state_.axis.value_or(kZAxis3f));
    state_.connected_up_axis = ComputeAxisAFromB(
        state_.connected_node, GetNode(), state_.up_axis.value_or(kYAxis3f));
  } else {
    connected_pivot = state_.connected_pivot.value_or(kZero3);
    connected_axis = state_.connected_axis.value_or(kZAxis3f);
    connected_up_axis = state_.connected_up_axis.value_or(kYAxis3f);
  }

  float3 xAxis = normalize(cross(state_.connected_up_axis.value_or(kYAxis3f),
                                 state_.connected_axis.value_or(kZAxis3f)));
  mat3f rot = mat3f(xAxis, state_.connected_up_axis.value_or(kYAxis3f),
                    state_.connected_axis.value_or(kZAxis3f));
  btTransform frame_in_A = ToBtTransform(
      state_.connected_pivot.value_or(kZero3), rot.toQuaternion());

  if (bt_rigid_body_B == nullptr) {
    // then we will attach it to the world
    bt_constraint_ = std::make_unique<btGeneric6DofConstraint>(
        *bt_rigid_body_A, frame_in_A, false);
  } else {
    xAxis = normalize(cross(state_.up_axis.value_or(kYAxis3f),
                            state_.axis.value_or(kZAxis3f)));
    rot = mat3f(xAxis, state_.up_axis.value_or(kYAxis3f),
                state_.axis.value_or(kZAxis3f));
    btTransform frame_in_B = ToBtTransform(state_.pivot, rot.toQuaternion());

    bt_constraint_ = std::make_unique<btGeneric6DofConstraint>(
        *bt_rigid_body_A, *bt_rigid_body_B, frame_in_A, frame_in_B, true);
  }

  state_.axis = axis;
  state_.up_axis = up_axis;
  state_.connected_pivot = connected_pivot;
  state_.connected_axis = connected_axis;
  state_.connected_up_axis = connected_up_axis;

  SetLimitsFromState();

  AddToPhysicsManager(true);

  return absl::OkStatus();
}

void Generic6DofConstraint::SetAngularLimits(float3 lower_limit,
                                             float3 upper_limit) {
  state_.angular_lower_limit = lower_limit;
  state_.angular_upper_limit = upper_limit;

  bt_constraint_->setAngularLowerLimit(
      ToBtVector3(state_.angular_lower_limit * ToRadians(1.0f)));
  bt_constraint_->setAngularUpperLimit(
      ToBtVector3(state_.angular_upper_limit * ToRadians(1.0f)));
}

void Generic6DofConstraint::SetLinearLimits(float3 lower_limit,
                                            float3 upper_limit) {
  state_.linear_lower_limit = lower_limit;
  state_.linear_upper_limit = upper_limit;

  bt_constraint_->setLinearLowerLimit(ToBtVector3(state_.linear_lower_limit));
  bt_constraint_->setLinearUpperLimit(ToBtVector3(state_.linear_upper_limit));
}

float3 Generic6DofConstraint::GetLowerLinearLimit() const {
  btVector3 lower_limit;
  bt_constraint_->getLinearLowerLimit(lower_limit);
  return ToFloat3(lower_limit);
}

float3 Generic6DofConstraint::GetUpperLinearLimit() const {
  btVector3 upper_limit;
  bt_constraint_->getLinearUpperLimit(upper_limit);
  return ToFloat3(upper_limit);
}

float3 Generic6DofConstraint::GetLowerAngularLimit() const {
  btVector3 lower_limit;
  bt_constraint_->getAngularLowerLimit(lower_limit);
  return ToFloat3(lower_limit) * ToDegrees(1.0f);
}

float3 Generic6DofConstraint::GetUpperAngularLimit() const {
  btVector3 upper_limit;
  bt_constraint_->getAngularUpperLimit(upper_limit);
  return ToFloat3(upper_limit) * ToDegrees(1.0f);
}

void Generic6DofConstraint::SetLimitsFromState() {
  bt_constraint_->setAngularLowerLimit(
      ToBtVector3(state_.angular_lower_limit * ToRadians(1.0f)));
  bt_constraint_->setAngularUpperLimit(
      ToBtVector3(state_.angular_upper_limit * ToRadians(1.0f)));

  bt_constraint_->setLinearLowerLimit(ToBtVector3(state_.linear_lower_limit));
  bt_constraint_->setLinearUpperLimit(ToBtVector3(state_.linear_upper_limit));
}

void Generic6DofConstraint::Update(const FrameTime& frame_time) {
  CheckIntegrityAndUpdate();
}

void Generic6DofConstraint::OnRigidBodiesChanged() {
  // delete the constraint and create it again
  AddToPhysicsManager(false);
  bt_constraint_.reset();

  absl::Status status = SetupInternal();
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to setup Point2PointConstraint: " << status;
  }
}

void Generic6DofConstraint::OnIsfStateChanged() {
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
