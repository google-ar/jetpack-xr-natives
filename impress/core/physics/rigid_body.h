/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_RIGID_BODY_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_RIGID_BODY_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "bullet/src/LinearMath/btDefaultMotionState.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/update_id.h"
#include "core/ncsb/update_phase.h"
#include "core/physics/collidable.h"
#include "core/physics/collidable_type.proto.imp.h"
#include "core/physics/physics_manager.h"
#include "core/physics/rigid_body_state.proto.imp.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/capsule_collider.h"
#include "core/view/framework/collision/cone_collider.h"
#include "core/view/framework/collision/cylinder_collider.h"
#include "core/view/framework/collision/mesh_collider.h"
#include "core/view/framework/collision/sphere_collider.h"
#include "core/view/utils/frame_time.h"
// TODO: Hide Bullet headers

namespace imp {

// A node with a RigidBody component is visible to the PhysicsManager.
// Object-to-object collision and gravity is enabled with this component.
class RigidBody : public Component {
 public:
  static constexpr UpdatePhase kUpdatePhase = PhysicsManager::kUpdatePhase;
  using UpdateDependencies = UpdateIds<PhysicsManager>;

  absl::Status Setup();
  absl::Status Setup(
      float mass,
      RigidBodyState::MotionMode mode = RigidBodyState::DIRECTED_DEFAULT,
      physics::CollidableType collidable_type = physics::AUTOMATIC_DEFAULT);
  absl::Status SetupWithState(const RigidBodyState& state);

  // Align the rigid body's property with backend.
  void Update(const FrameTime& frame_time);

  // Set the object to be simulated / directed.
  void SetAsSimulated(bool simulated);

  // Unit: Kg.
  void SetMass(float mass);

  // Sets up coefficient of sliding friction.
  void SetFriction(float friction);

  // Sets up coefficient of restitution. O means no bounce (plastic), 1 means
  // perfect bounce(elastic).
  void SetRestitution(float restitution);

  // Set's the rigid body's linear velocity. This will take effect on the next
  // frame.
  void SetLinearVelocity(float3 velocity);

  // Set's the rigid body's linear factor - how much it will move on x, y, z
  // axis. Can be used for freezing the object on one axis.
  void SetLinearFactor(const float3& linear_factor);

  // Get's the rigid body's current linear factor.
  float3 GetLinearFactor();

  // Set's the rigid body's angular velocity. This will take effect on the next
  // frame.
  void SetAngularVelocity(float3 angular_velocity);

  // Get's the rigid body's current linear velocity.
  float3 GetLinearVelocity();

  // Get's the rigid body's current angular velocity.
  float3 GetAngularVelocity();

  // Set's the rigid body's angular factor.
  void SetAngularFactor(float3 angular_factor);

  // Get's the rigid body's current angular factor.
  float3 GetAngularFactor();

  // Sets the gravity that the object is subject to.
  void SetCustomGravity(float3 gravity);

  // Clear custom gravity and use world gravity.
  void UseWorldGravity();

  // Adds a force through the geometric center of the object's collider, in
  // Newtons. Last for 1 frame.
  void ApplyForce(const float3& force);

  // Adds a force at a relative position from the geometric center of the
  // object's collider, in Newtons. When relative_position is non-zero, results
  // in a central force and a torque. Last for 1 frame.
  void ApplyForce(const float3& force, const float3& relative_position);

  // Adds an impulse to the object, in Newton-seconds (Ns). This will change
  // the object's velocity.
  void ApplyImpulse(const float3& impulse);

  // Adds a force at a relative position from the geometric center of the
  // object's collider, in Newton-seconds (Ns). When relative_position is
  // non-zero, results in a central impulse and a torque impulse.
  void ApplyImpulse(const float3& impulse, const float3& relative_position);

  // Adds a torque, where a positive value indicates a counter-clockwise torque
  // around that axis, in Newton-meters (Nm). Last for 1 frame.
  void ApplyTorque(const float3& torque);

  float GetMass() const { return state_.mass; }

  RigidBodyState::MotionMode GetMotionMode() const {
    return state_.motion_mode;
  }

  void OnIsfStateChanged();

  void OnActiveStatusChanged(bool is_active);

  const btRigidBody& GetBtRigidBody() const { return *rigid_body_; }

  void Cleanup();

  // Returns the position and size (but not orientation) of the collision shape,
  // for testing the alignment between Impress and Bullet colliders.
  Collidable::CollisionShape GetCollisionShape() const;

 private:
  absl::Status InitializeDirected(const btTransform& bt_transform);
  absl::Status InitializeSimulated(const btTransform& bt_transform);
  absl::Status InitializeNonMovable();

  // Creates a directed object from existing simulated objects.
  void SwitchToDirectedInternally();

  // Sets the friction, uses default value if not provided.
  void SetFrictionInternal(absl::optional<float> friction);

  PhysicsManager* physics_manager_;
  Collidable collidable_;
  std::unique_ptr<btRigidBody> rigid_body_;
  std::unique_ptr<btDefaultMotionState> motion_state_;

  RigidBodyState state_;

  // Is temporarily directed by the user, but being a simulated object.
  bool was_directed_while_simulated_ = false;
  mat4f transform_prev_;

 public:
  using IsfInfo =
      IsfInfo<&RigidBody::state_,
              IsfDependencies<SphereCollider, BoxCollider, CapsuleCollider,
                              CylinderCollider, ConeCollider, MeshCollider,
                              GltfRenderer>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_RIGID_BODY_H_
