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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_PHYSICS_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_PHYSICS_MANAGER_H_

#include <cstddef>

#include "absl/container/flat_hash_map.h"
#include "bullet/src/BulletCollision/BroadphaseCollision/btAxisSweep3.h"
#include "bullet/src/BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "bullet/src/BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "bullet/src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "bullet/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "bullet/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "core/common/invocable.h"
#include "core/common/robin_map.h"
#include "core/common/robin_set.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_phase.h"
#include "core/ncsb/update_system.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp {
// Physics Manager holds a DiscreteDynamicsWorld, a Bullet Physics instance that
// simulates a world subject to physics laws.
//
// Example: in RigidBody component
// (Register your rigid body through physics manager)
// physics_manager->AddRigidBody(rigid_body);
// (Remember to unregister in component Cleanup())
//
// After registration, the "rigid_body" is now visible to the physics world. In
// the physics world, objects can collide with each other. They are subject to
// gravity (unless marked as static). The speeds and positions of those objects
// may change over time.
//
// TODO: record binary impact of this using Bullet.
class PhysicsManager : public UpdateSystem::Updater<PhysicsManager> {
 public:
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kPreDefault;

  // Records both parties of a physics based object to object collision.
  struct CollisionEvent : Event {
    CollisionEvent(NodeHandle first, NodeHandle second)
        : first(first), second(second) {}

    // The node that receives this event.
    NodeHandle first;
    // The other node in the collision.
    NodeHandle second;
  };

  explicit PhysicsManager(BaseView& view);

  // Runs simulation of the physics.
  void Update(const FrameTime& frame_time) override;

  // Adds the rigid body to physics world. Keeps track of the node that the
  // rigid body component is attached to.
  void AddRigidBody(btRigidBody& body, NodeHandle node);

  // Removes the rigid body from the physics world.
  void RemoveRigidBody(btRigidBody& body);

  // Adds the trigger volume to physics world. Keeps track of the node that the
  // trigger volume component is attached to.
  void AddTriggerVolume(btCollisionObject& body, NodeHandle node);

  // Removes the trigger volume from the physics world.
  void RemoveTriggerVolume(btCollisionObject& body);

  // Adds the constraint to the physics world.
  void AddConstraint(btTypedConstraint& constraint);

  // Removes the constraint from the physics world
  void RemoveConstraint(btTypedConstraint& constraint);

  // Checks if the constraint is in the physics world.
  // It can be removed by a RigidBody removal.
  bool IsConstraintActive(btTypedConstraint& constraint) const;

  // Sets the gravity of the physics world. Unit: Newton per kg. Default is (0,
  // -9.80625, 0).
  void SetWorldGravity(float3 gravity);

  // Returns the gravity of the physics world. Unit: Newton per kg.
  float3 GetWorldGravity() const;

  // Returns the number of objects that are participating in the physics
  // simulation.
  size_t GetNumberOfObjects() const;

  // Sets the play state of the physics simulation. True to play, false to
  // pause. The simulation is playing by default.
  void PlaySimulation(bool play);

  // Sets a custom step speed for the physics simulation. < 1.0f
  // will slow down the simulation, > 1.0f will speed it up. 1.0f is the
  // default.
  void SetSimulationStepSpeed(float speed);

  // Returns the current step speed for the physics simulation.
  float GetSimulationStepSpeed() const;

  // Advances the simulation over some duration. This will have no effect if the
  // simulation is playing.
  void FastForwardSimulation(float duration);

#if IMP_RUNTIME(DEV)
  // Registers a visualizer for a collidable. Based on the assumption that one
  // Node can only have one collidable.
  void RegisterCollidableVisualizer(NodeHandle node,
                                    imp::Invocable<void()> visualizer);

  // Unregisters a visualizer for a collidable. Based on the assumption that one
  // Node can only have one collidable.
  void UnregisterCollidableVisualizer(NodeHandle node);

  void DrawCollidables();
#endif

 private:
  // Sends collision events to collided nodes.
  void ProcessCollisions();

  // Find out whether there is an active collision object associated with the
  // node.
  bool HasActiveCollidables(NodeHandle node) const;

  BaseView& view_;
  btDefaultCollisionConfiguration collision_configuration_;
  btCollisionDispatcher physics_dispatcher_;
  btAxisSweep3 aabb_overlapping_pair_cache_;
  btSequentialImpulseConstraintSolver solver_;
  btDiscreteDynamicsWorld world_;
  RobinMap<btCollisionObject*, NodeHandle> rigid_body_map_;
  RobinMap<btRigidBody*, RobinSet<btTypedConstraint*>>
      constraint_dependency_map_;
  RobinSet<btTypedConstraint*> active_constraints_map_;
  bool play_simulation_;
  float simulation_step_speed_;

#if IMP_RUNTIME(DEV)
  absl::flat_hash_map<NodeHandle, imp::Invocable<void()>>
      collidable_visualizer_map_;
#endif
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_PHYSICS_MANAGER_H_
