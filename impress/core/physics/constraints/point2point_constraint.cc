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

#include "core/physics/constraints/point2point_constraint.h"

#include <memory>
#include <optional>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "bullet/src/LinearMath/btVector3.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/constraints/point2point_constraint_state.proto.imp.h"
#include "core/physics/physics_helper.h"
#include "core/view/utils/frame_time.h"

namespace imp {

absl::Status Point2PointConstraint::Setup() { return SetupInternal(); }

absl::Status Point2PointConstraint::Setup(NodeHandle connected_node,
                                          float3 pivot, bool auto_configure,
                                          float3 connected_pivot) {
  state_.connected_node.AssignSceneHandleForNode(connected_node);
  state_.auto_configure = auto_configure;
  state_.pivot = pivot;
  state_.connected_pivot = connected_pivot;

  return SetupInternal();
}

absl::Status Point2PointConstraint::SetupWithState() { return SetupInternal(); }

absl::Status Point2PointConstraint::SetupInternal() {
  state_.is_ready = true;
  if (InitializeWithNodes(state_.connected_node, GetNode()) !=
      absl::OkStatus()) {
    state_.is_ready = false;
  }

  if (!state_.is_ready) {
    bt_constraint_.reset();
    return absl::OkStatus();
  }

  float3 connected_pivot;

  btRigidBody* bt_rigid_body_A =
      const_cast<btRigidBody*>(&GetRigidBodyA()->GetBtRigidBody());
  btRigidBody* bt_rigid_body_B =
      GetRigidBodyB() == nullptr
          ? nullptr
          : const_cast<btRigidBody*>(&GetRigidBodyB()->GetBtRigidBody());

  if (state_.auto_configure || bt_rigid_body_B == nullptr) {
    connected_pivot =
        ComputePivotAFromB(state_.connected_node, GetNode(), state_.pivot);
  } else {
    connected_pivot = state_.connected_pivot.value_or(kZero3);
  }

  btVector3 bt_pivot_in_A = ToBtVector3(connected_pivot);
  if (bt_rigid_body_B == nullptr) {
    // then we will attach it to the world
    bt_constraint_ = std::make_unique<btPoint2PointConstraint>(*bt_rigid_body_A,
                                                               bt_pivot_in_A);
  } else {
    btVector3 bt_pivot_in_B = ToBtVector3(state_.pivot);

    bt_constraint_ = std::make_unique<btPoint2PointConstraint>(
        *bt_rigid_body_A, *bt_rigid_body_B, bt_pivot_in_A, bt_pivot_in_B);
  }

  state_.connected_pivot = connected_pivot;

  AddToPhysicsManager(true);

  return absl::OkStatus();
}

void Point2PointConstraint::Cleanup() { AddToPhysicsManager(false); }

void Point2PointConstraint::Update(const FrameTime& frame_time) {
  CheckIntegrityAndUpdate();
}

void Point2PointConstraint::OnRigidBodiesChanged() {
  // delete the constraint and create it again
  AddToPhysicsManager(false);
  bt_constraint_.reset();

  absl::Status status = SetupInternal();
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to setup Point2PointConstraint: " << status;
  }
}

void Point2PointConstraint::OnIsfStateChanged() {
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
