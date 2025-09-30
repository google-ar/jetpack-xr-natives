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

#include "core/physics/constraints/base_constraint.h"

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/common/registry.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/physics_manager.h"
#include "core/physics/rigid_body.h"
#include "core/view/framework/collision/collision_manager.h"

namespace imp {

void BaseConstraint::AddToPhysicsManager(bool add) {
  if (add) {
    if (!added_to_physics_manager_) {
      physics_manager_->AddConstraint(*GetBtConstraint());
      added_to_physics_manager_ = true;
    }
  } else {
    if (added_to_physics_manager_) {
      if (GetBtConstraint()) {
        physics_manager_->RemoveConstraint(*GetBtConstraint());
      }
      added_to_physics_manager_ = false;
    }
  }
}

absl::Status BaseConstraint::InitializeWithNodes(NodeHandle connected_node,
                                                 NodeHandle owner_node) {
  if (!physics_manager_) {
    BaseView& view = owner_node->GetView();
    physics_manager_ = &view.GetRegistry().GetOrCreate<PhysicsManager>(view);
  }

  connected_node_ = connected_node;
  owner_node_ = owner_node;

  RigidBody* rigid_body_a_ptr = GetRigidBodyA();

  if (!rigid_body_a_ptr) {
    return absl::FailedPreconditionError(
        "Rigidbody A (connected node) is invalid/inactive");
  }

  connected_node_valid_prev_ = GetRigidBodyA() != nullptr;
  owner_node_valid_prev_ = GetRigidBodyB() != nullptr;

  return absl::OkStatus();
}

RigidBody* BaseConstraint::GetRigidBodyA() {
  if (connected_node_.IsValid()) {
    ComponentHandle<RigidBody> rigid_body =
        connected_node_->GetComponent<RigidBody>();
    if (rigid_body.IsValid() && rigid_body->IsActive()) {
      return rigid_body.Get();
    }
  }
  return nullptr;
}

RigidBody* BaseConstraint::GetRigidBodyB() {
  ComponentHandle<RigidBody> rigid_body =
      owner_node_->GetComponent<RigidBody>();
  if (rigid_body.IsValid() && rigid_body->IsActive()) {
    return rigid_body.Get();
  }
  return nullptr;
}

float3 BaseConstraint::ComputePivotAFromB(NodeHandle node_a, NodeHandle node_b,
                                          const float3& pivot_in_b) {
  float3 p = pivot_in_b;
  if (node_b->GetComponent<RigidBody>()) {
    p = node_b->WorldFromLocalPoint(p);
  }
  p = node_a->LocalFromWorldPoint(p);
  return p;
}

bool BaseConstraint::IsActiveInWorld() const {
  if (physics_manager_ && GetBtConstraint()) {
    return physics_manager_->IsConstraintActive(*GetBtConstraint());
  }
  return false;
}

void BaseConstraint::CheckIntegrityAndUpdate() {
  // The constraint may be removed from the physics world by a rigid body
  // removal.
  if (!IsActiveInWorld()) {
    added_to_physics_manager_ = false;
  }

  // TODO: (broken link) - We should test here also the motion mode of the
  // rigid bodies, because both DIRECTED would break the constraint. Also test
  // for NON_MOVABLE.
  bool connected_node_valid = GetRigidBodyA() != nullptr;
  bool owner_node_valid = GetRigidBodyB() != nullptr;
  if (connected_node_valid_prev_ != connected_node_valid ||
      owner_node_valid_prev_ != owner_node_valid) {
    connected_node_valid_prev_ = connected_node_valid;
    owner_node_valid_prev_ = owner_node_valid;
    OnRigidBodiesChanged();
  }
}

}  // namespace imp
