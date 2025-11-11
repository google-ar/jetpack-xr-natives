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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_TRIGGER_VOLUME_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_TRIGGER_VOLUME_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "bullet/src/BulletCollision/CollisionDispatch/btGhostObject.h"
#include "core/config.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/physics/collidable.h"
#include "core/physics/physics_manager.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/capsule_collider.h"
#include "core/view/framework/collision/compound_collider.h"
#include "core/view/framework/collision/cone_collider.h"
#include "core/view/framework/collision/cylinder_collider.h"
#include "core/view/framework/collision/sphere_collider.h"
#include "core/view/utils/frame_time.h"
// TODO: Hide Bullet headers

namespace imp {

// A node with a TriggerVolume component is visible to the PhysicsManager.
// Objects with RigidBody components that enters the trigger volume will be
// detected and reported.
//
// Example:
//
// NodeHandle node = CreateNode();
// // Define the shape of the trigger volume with an Impress collider.
// node->AddComponent<SphereCollider>();
//
// auto trigger_volume = node->AddComponent<TriggerVolume>();
// node->Connect([](const PhysicsManager::CollisionEvent& event) {
//   // event.second is the node that entered the trigger volume.
// });
class TriggerVolume : public Component {
 public:
  absl::Status Setup();

  // Align the trigger volume's property with backend.
  void Update(const FrameTime& frame_time);

  void Cleanup();

  // Returns the position and size (but not orientation) of the collision shape,
  // for testing the alignment between Impress and Bullet colliders.
  Collidable::CollisionShape GetCollisionShape() const;

 private:
  std::unique_ptr<btGhostObject> trigger_volume_;
  Collidable collidable_;
  PhysicsManager* physics_manager_;

  static constexpr absl::string_view kType = "imp.Physics.TriggerVolume";

#if IMP_RUNTIME(DEV)
  void Visualize();
#endif

 public:
  using IsfInfo = StatelessIsfInfo<
      TriggerVolume, kType,
      IsfDependencies<SphereCollider, BoxCollider, CapsuleCollider,
                      CylinderCollider, ConeCollider, CompoundCollider>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_TRIGGER_VOLUME_H_
