// Copyright 2024 Google LLC
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

#include "core/physics/rigid_body.h"

#include <cstddef>
#include <memory>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "bullet/src/BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "bullet/src/LinearMath/btDefaultMotionState.h"
#include "bullet/src/LinearMath/btScalar.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "bullet/src/LinearMath/btVector3.h"
#include "core/common/registry.h"
#include "core/config.h"
#include "core/math/almost_equal.h"
#include "core/math/mat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/physics/collidable.h"
#include "core/physics/collidable_type.proto.imp.h"
#include "core/physics/physics_helper.h"
#include "core/physics/physics_manager.h"
#include "core/physics/rigid_body_state.proto.imp.h"
#include "core/physics/trigger_volume.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"
#include "mediapipe/framework/port/status_macros.h"
#if IMP_RUNTIME(DEV)
#include "core/common/debug_draw.h"
#endif

namespace imp {

// A simulated object cannot have no mass. A small mass value will be assigned
// if there is no valid mass value provided.
static constexpr float kSimulatedMinimalMass = 0.01f;
static constexpr float kDefaultFriction = 0.5f;
static constexpr float kSimulatedMinimumVelocity = 0.001f;

absl::Status RigidBody::Setup(float mass, RigidBodyState::MotionMode mode,
                              physics::CollidableType collidable_type) {
  state_.mass = mass;
  state_.motion_mode = mode;
  state_.collidable_type = collidable_type;
  return Setup();
}

absl::Status RigidBody::Setup() {
  if (GetNode()->GetComponent<TriggerVolume>()) {
    return absl::FailedPreconditionError(
        "Cannot add a RigidBody component,"
        "since the collider is associated with a TriggerVolume component.");
  }

  physics_manager_ =
      &GetView().GetRegistry().GetOrCreate<PhysicsManager>(GetView());

  btTransform start_transform;
  MP_RETURN_IF_ERROR(
      collidable_.Create(start_transform, GetNode(), state_.collidable_type));
  state_.collidable_type = collidable_.GetCollidableType();
  if (!collidable_.IsMovable()) {
    state_.motion_mode = RigidBodyState::NONMOVABLE;
    return InitializeNonMovable();
  }

  if (state_.motion_mode == RigidBodyState::SIMULATED) {
    return InitializeSimulated(start_transform);
  } else {
    return InitializeDirected(start_transform);
  }
}

absl::Status RigidBody::SetupWithState(const RigidBodyState& state) {
  state_ = state;
  return Setup();
}

absl::Status RigidBody::InitializeNonMovable() {
  transform_prev_ = GetNode()->GetWorldTrs();

  btRigidBody::btRigidBodyConstructionInfo rb_info(
      0.0f, nullptr, collidable_.GetBtCollisionShape(), btVector3(0, 0, 0));

  rigid_body_ = std::make_unique<btRigidBody>(rb_info);
  rigid_body_->setActivationState(DISABLE_DEACTIVATION);

  if (IsActive()) {
    physics_manager_->AddRigidBody(*rigid_body_, GetNode());
  }

  return absl::OkStatus();
}

absl::Status RigidBody::InitializeDirected(const btTransform& bt_transform) {
  size_t directed_mass = state_.mass;
  MP_RETURN_IF_ERROR(InitializeSimulated(bt_transform));

  SwitchToDirectedInternally();
  state_.mass = directed_mass;

  return absl::OkStatus();
}

absl::Status RigidBody::InitializeSimulated(const btTransform& bt_transform) {
  // Remove old btRigidBody object if already exists.
  Cleanup();

  transform_prev_ = GetNode()->GetWorldTrs();

  if (AlmostEqual(state_.mass, 0.0f)) {
    state_.mass = kSimulatedMinimalMass;
  }
  btScalar bt_mass(state_.mass);
  btVector3 local_inertia(0, 0, 0);
  // TODO: Add test for local_inertia.
  collidable_.GetBtCollisionShape()->calculateLocalInertia(bt_mass,
                                                           local_inertia);

  motion_state_ = std::make_unique<btDefaultMotionState>(bt_transform);
  btRigidBody::btRigidBodyConstructionInfo rb_info(
      bt_mass, motion_state_.get(), collidable_.GetBtCollisionShape(),
      local_inertia);

  rigid_body_ = std::make_unique<btRigidBody>(rb_info);
  SetFrictionInternal(state_.friction);
  SetRestitution(state_.restitution);

  if (length(state_.linear_velocity) > kSimulatedMinimumVelocity) {
    SetLinearVelocity(state_.linear_velocity);
  }
  if (length(state_.angular_velocity) > kSimulatedMinimumVelocity) {
    SetAngularVelocity(state_.angular_velocity);
  }

  // Without DISABLE_DEACTIVATION, object gets deactivated if it is not moving.
  // Disables this performance optimization for now.
  rigid_body_->setActivationState(DISABLE_DEACTIVATION);

  if (state_.gravity.has_value()) {
    SetCustomGravity(*state_.gravity);
  }

  if (state_.linear_factor.has_value()) {
    SetLinearFactor(*state_.linear_factor);
  }

  if (state_.angular_factor.has_value()) {
    SetAngularFactor(*state_.angular_factor);
  }

  if (IsActive()) {
    physics_manager_->AddRigidBody(*rigid_body_, GetNode());
  }
  return absl::OkStatus();
}

void RigidBody::SwitchToDirectedInternally() {
  rigid_body_->setMassProps(0, btVector3(0, 0, 0));
}

void RigidBody::SetAsSimulated(bool simulated) {
  if (simulated) {
    if (!rigid_body_->isStaticOrKinematicObject()) return;
    btTransform bt_transform = collidable_.GetNodeBtTransform();
    // Make a fresh simulated object.
    auto status = InitializeSimulated(bt_transform);
    state_.motion_mode = RigidBodyState::SIMULATED;
  } else {
    SwitchToDirectedInternally();
    state_.motion_mode = RigidBodyState::DIRECTED_DEFAULT;
  }
}

void RigidBody::SetMass(float mass) {
  if (!AlmostEqual(mass, 0.0f)) {
    state_.mass = mass;
  } else {
    state_.mass = kSimulatedMinimalMass;
  }
  btVector3 local_inertia(0, 0, 0);
  collidable_.GetBtCollisionShape()->calculateLocalInertia(state_.mass,
                                                           local_inertia);
  rigid_body_->setMassProps(state_.mass, local_inertia);
}

void RigidBody::SetFriction(float friction) { SetFrictionInternal(friction); }

void RigidBody::SetFrictionInternal(absl::optional<float> friction) {
  if (!friction.has_value()) {
    friction = kDefaultFriction;
  }
  rigid_body_->setFriction(*friction);
  state_.friction = friction;
}

void RigidBody::SetRestitution(float restitution) {
  rigid_body_->setRestitution(restitution);
  state_.restitution = restitution;
}

void RigidBody::SetLinearVelocity(float3 velocity) {
  state_.linear_velocity = velocity;
  rigid_body_->setLinearVelocity(ToBtVector3(velocity));
}

void RigidBody::SetLinearFactor(const float3& linear_factor) {
  state_.linear_factor = linear_factor;
  rigid_body_->setLinearFactor(ToBtVector3(linear_factor));
}

float3 RigidBody::GetLinearFactor() {
  return ToVec3<float>(rigid_body_->getLinearFactor());
}

void RigidBody::SetAngularVelocity(float3 angular_velocity) {
  state_.angular_velocity = angular_velocity;
  rigid_body_->setAngularVelocity(ToBtVector3(angular_velocity));
}

float3 RigidBody::GetLinearVelocity() {
  return ToVec3<float>(rigid_body_->getLinearVelocity());
}

float3 RigidBody::GetAngularVelocity() {
  return ToVec3<float>(rigid_body_->getAngularVelocity());
}

void RigidBody::SetAngularFactor(float3 angular_factor) {
  state_.angular_factor = angular_factor;
  rigid_body_->setAngularFactor(ToBtVector3(angular_factor));
}

float3 RigidBody::GetAngularFactor() {
  return ToVec3<float>(rigid_body_->getAngularFactor());
}

void RigidBody::SetCustomGravity(float3 gravity) {
  rigid_body_->setFlags(btRigidBodyFlags::BT_DISABLE_WORLD_GRAVITY);
  rigid_body_->setGravity(ToBtVector3(gravity));
  state_.gravity = gravity;
}

void RigidBody::UseWorldGravity() {
  rigid_body_->setFlags(!btRigidBodyFlags::BT_DISABLE_WORLD_GRAVITY);
  rigid_body_->setGravity(ToBtVector3(physics_manager_->GetWorldGravity()));
  state_.gravity.reset();
}

void RigidBody::ApplyForce(const float3& force) {
  rigid_body_->applyCentralForce(ToBtVector3(force));
}

void RigidBody::ApplyForce(const float3& force,
                           const float3& relative_position) {
  rigid_body_->applyForce(ToBtVector3(force), ToBtVector3(relative_position));
}

void RigidBody::ApplyImpulse(const float3& impulse) {
  rigid_body_->applyCentralImpulse(ToBtVector3(impulse));
}

void RigidBody::ApplyImpulse(const float3& impulse,
                             const float3& relative_position) {
  rigid_body_->applyImpulse(ToBtVector3(impulse),
                            ToBtVector3(relative_position));
}

void RigidBody::ApplyTorque(const float3& torque) {
  rigid_body_->applyTorque(ToBtVector3(torque));
}

void RigidBody::OnIsfStateChanged() {
  // state_.collidable_type cannot be changed after Setup.
  state_.collidable_type = collidable_.GetCollidableType();
  if (state_.gravity.has_value()) {
    SetCustomGravity(*state_.gravity);
  } else {
    UseWorldGravity();
  }

  if (state_.linear_factor.has_value()) {
    SetLinearFactor(*state_.linear_factor);
  } else {
    rigid_body_->setLinearFactor(btVector3(1, 1, 1));
  }

  if (state_.angular_factor.has_value()) {
    SetAngularFactor(*state_.angular_factor);
  } else {
    rigid_body_->setAngularFactor(btVector3(1, 1, 1));
  }

  SetMass(state_.mass);
  SetFrictionInternal(state_.friction);
  SetRestitution(state_.restitution);

  if (!collidable_.IsMovable()) {
    state_.motion_mode = RigidBodyState::MotionMode::NONMOVABLE;

    return;
  }

  // Only use NONMOVABLE mode for objects that are not movable.
  if (state_.motion_mode == RigidBodyState::NONMOVABLE &&
      collidable_.IsMovable()) {
    state_.motion_mode = RigidBodyState::DIRECTED_DEFAULT;
  }

  if (state_.motion_mode == RigidBodyState::DIRECTED_DEFAULT) {
    SetAsSimulated(false);
  } else {
    SetAsSimulated(true);
  }
}

void RigidBody::OnActiveStatusChanged(bool is_active) {
  if (is_active) {
    if (state_.motion_mode == RigidBodyState::DIRECTED_DEFAULT) {
      btTransform bt_transform;
      rigid_body_->getMotionState()->getWorldTransform(bt_transform);
      auto status = InitializeDirected(bt_transform);
    } else {
      physics_manager_->AddRigidBody(*rigid_body_, GetNode());
    }
  } else {
    Cleanup();
  }
}

void RigidBody::Cleanup() {
  if (rigid_body_) {
    physics_manager_->RemoveRigidBody(*rigid_body_);
  }
}

void RigidBody::Update(const FrameTime& frame_time) {
  if (!collidable_.IsMovable()) {
    if (!AlmostEqual(GetNode()->GetWorldTrs(), transform_prev_)) {
      GetNode()->SetWorldTrs(transform_prev_);
      IMP_LOG(imp::ERROR) << GetNode()->GetName() << " is static.";
    }
#if IMP_RUNTIME(DEV)
    collidable_.Visualize({});
#endif
    return;
  }

  btTransform bt_transform = collidable_.GetNodeBtTransform();

  if (state_.motion_mode == RigidBodyState::SIMULATED) {
    if (!AlmostEqual(transform_prev_, GetNode()->GetWorldTrs())) {
      // Switch to directed object when begin to be directed by the user.
      if (!was_directed_while_simulated_) {
        SwitchToDirectedInternally();
        was_directed_while_simulated_ = true;
      }
      rigid_body_->getMotionState()->setWorldTransform(bt_transform);
      collidable_.ApplyScalingToBulletCollider();
    } else {
      if (was_directed_while_simulated_) {
        // Switch back to simulated object when no longer directed by the user.
        collidable_.ApplyScalingToBulletCollider();
        auto status = InitializeSimulated(bt_transform);
        was_directed_while_simulated_ = false;
      } else {
        // Pull the position from physics backend.
        btTransform bt_trans;
        rigid_body_->getMotionState()->getWorldTransform(bt_trans);
        Transform<float> transform = ToTransform(bt_trans);
        transform.scale =
            ToVec3<float>(collidable_.GetBtCollisionShape()->getLocalScaling());
        float3 center_offset =
            transform.rotation * collidable_.GetCollidableCenter();
        transform.translation -= center_offset * transform.scale;
        GetNode()->SetWorldTrs(transform.AsMat4());
      }
    }
  } else {
    // Update Bullet collider's transformation.
    if (!AlmostEqual(transform_prev_, GetNode()->GetWorldTrs())) {
      rigid_body_->setWorldTransform(bt_transform);
      rigid_body_->getMotionState()->setWorldTransform(bt_transform);
    }
    collidable_.ApplyScalingToBulletCollider();
  }

  transform_prev_ = GetNode()->GetWorldTrs();
  state_.linear_velocity = GetLinearVelocity();
  state_.angular_velocity = GetAngularVelocity();

#if IMP_RUNTIME(DEV)
  btTransform current_bt_transform;
  rigid_body_->getMotionState()->getWorldTransform(current_bt_transform);
  collidable_.Visualize(current_bt_transform);
#endif
}

Collidable::CollisionShape RigidBody::GetCollisionShape() const {
  btTransform bt_trans;
  rigid_body_->getMotionState()->getWorldTransform(bt_trans);
  return collidable_.GetCollisionShape(bt_trans);
}

}  // namespace imp
