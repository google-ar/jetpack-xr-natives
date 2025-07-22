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

#include "core/physics/physics_manager.h"

#include <algorithm>
#include <cstddef>

#include "bullet/src/BulletCollision/BroadphaseCollision/btAxisSweep3.h"
#include "bullet/src/BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "bullet/src/BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "bullet/src/BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"
#include "bullet/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "bullet/src/LinearMath/btVector3.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_system.h"
#include "core/physics/physics_helper.h"
#include "core/physics/rigid_body.h"
#include "core/physics/trigger_volume.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// The half extent of the cube space that underlying physics engine will work.
static constexpr float kPhysicsWorldHalfExtent = 1000.0f;
// The threshold for the distance between two objects, below which is
// considered as a collision.
static constexpr float kCollisionDistanceThreshold = 0.001f;
// The max time step length for a single physics step.
static constexpr float kMaxTimeStepLength = 0.015f;

PhysicsManager::PhysicsManager(BaseView& view)
    : Updater(view),
      view_(view),
      physics_dispatcher_(btCollisionDispatcher(&collision_configuration_)),
      aabb_overlapping_pair_cache_(btAxisSweep3(
          btVector3(-kPhysicsWorldHalfExtent, -kPhysicsWorldHalfExtent,
                    -kPhysicsWorldHalfExtent),
          btVector3(kPhysicsWorldHalfExtent, kPhysicsWorldHalfExtent,
                    kPhysicsWorldHalfExtent))),
      world_(btDiscreteDynamicsWorld(&physics_dispatcher_,
                                     &aabb_overlapping_pair_cache_, &solver_,
                                     &collision_configuration_)),
      play_simulation_(true),
      simulation_step_speed_(1.0f) {
  // TODO: Make gravity configurable.
  world_.setGravity(btVector3(0., -9.80625, 0.));
}

void PhysicsManager::Update(const FrameTime& frame_time) {
  if (!play_simulation_) {
    return;
  }

  world_.stepSimulation(frame_time.GetDeltaSeconds() * simulation_step_speed_);

  ProcessCollisions();
}

void PhysicsManager::AddRigidBody(btRigidBody& body, NodeHandle node) {
  world_.addRigidBody(&body);
  rigid_body_map_[&body] = node;
}

void PhysicsManager::RemoveRigidBody(btRigidBody& body) {
  world_.removeRigidBody(&body);
  rigid_body_map_.erase(&body);
}

void PhysicsManager::AddTriggerVolume(btCollisionObject& body,
                                      NodeHandle node) {
  world_.addCollisionObject(&body);
  rigid_body_map_[&body] = node;
}

void PhysicsManager::RemoveTriggerVolume(btCollisionObject& body) {
  world_.removeCollisionObject(&body);
  rigid_body_map_.erase(&body);
}

void PhysicsManager::SetWorldGravity(float3 gravity) {
  world_.setGravity(ToBtVector3(gravity));
}

float3 PhysicsManager::GetWorldGravity() const {
  return ToVec3<float>(world_.getGravity());
}

size_t PhysicsManager::GetNumberOfObjects() const {
  return world_.getNumCollisionObjects();
}

void PhysicsManager::ProcessCollisions() {
  for (size_t i = 0; i < physics_dispatcher_.getNumManifolds(); i++) {
    // A contact manifold represents a pair of objects that are potentially
    // colliding with each other.
    btPersistentManifold* contactManifold =
        physics_dispatcher_.getManifoldByIndexInternal(i);

    // Between the pair of objects, there can be multiple potential contact
    // points.
    bool is_collided = false;
    for (size_t p = 0; p < contactManifold->getNumContacts(); p++) {
      btManifoldPoint& pt = contactManifold->getContactPoint(p);
      // Two objects are considered collided only when the distance between them
      // is small than the threshold.
      if (pt.getDistance() < kCollisionDistanceThreshold) {
        is_collided = true;
        break;
      }
    }
    if (!is_collided) continue;

    NodeHandle node_0 = rigid_body_map_[const_cast<btCollisionObject*>(
        contactManifold->getBody0())];
    NodeHandle node_1 = rigid_body_map_[const_cast<btCollisionObject*>(
        contactManifold->getBody1())];

    if (HasActiveCollidables(node_0) && HasActiveCollidables(node_1)) {
      node_0->Send(CollisionEvent(node_0, node_1));
      node_1->Send(CollisionEvent(node_1, node_0));
    }
  }
}

bool PhysicsManager::HasActiveCollidables(NodeHandle node) const {
  auto rigid_body = node->GetComponent<RigidBody>();
  if (rigid_body && rigid_body->IsActive()) {
    return true;
  }

  auto trigger_volume = node->GetComponent<TriggerVolume>();
  if (trigger_volume && trigger_volume->IsActive()) {
    return true;
  }

  return false;
}

void PhysicsManager::PlaySimulation(bool play) { play_simulation_ = play; }

void PhysicsManager::SetSimulationStepSpeed(float speed) {
  simulation_step_speed_ = std::max(0.0f, speed);
}

float PhysicsManager::GetSimulationStepSpeed() const {
  return simulation_step_speed_;
}

void PhysicsManager::FastForwardSimulation(float duration) {
  if (play_simulation_ || duration <= 0.0f) {
    return;
  }

  if (duration > kMaxTimeStepLength) {
    float accumlated_time_step = 0.0f;
    while (accumlated_time_step < duration) {
      if (accumlated_time_step + kMaxTimeStepLength > duration) {
        float remaining_time_step = duration - accumlated_time_step;
        world_.stepSimulation(remaining_time_step);
        break;
      }
      world_.stepSimulation(kMaxTimeStepLength);
      accumlated_time_step += kMaxTimeStepLength;
    }
  } else {
    world_.stepSimulation(duration);
  }
}

}  // namespace imp
