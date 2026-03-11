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

#include "core/physics/trigger_volume.h"

#include <memory>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "bullet/src/BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "bullet/src/BulletCollision/CollisionDispatch/btGhostObject.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/common/registry.h"
#include "core/config.h"
#include "core/physics/collidable.h"
#include "core/physics/physics_manager.h"
#include "core/physics/rigid_body.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"
#include "mediapipe/framework/port/status_macros.h"
#if IMP_RUNTIME(DEV)
#include "core/common/debug_draw.h"
#include "core/physics/physics_constants.h"
#endif

namespace imp {

absl::Status TriggerVolume::Setup() {
  if (GetNode()->GetComponent<RigidBody>()) {
    return absl::FailedPreconditionError(
        "Cannot add a TriggerVolume component,"
        "since the collider is associated with a RigidBody component.");
  }

  btTransform start_transform;
  MP_RETURN_IF_ERROR(collidable_.Create(
      /*output*/ start_transform, /*input*/ GetNode()));

  trigger_volume_ = std::make_unique<btGhostObject>();
  trigger_volume_->setCollisionShape(collidable_.GetBtCollisionShape());
  trigger_volume_->setCollisionFlags(trigger_volume_->getCollisionFlags() |
                                     btCollisionObject::CF_NO_CONTACT_RESPONSE);
  trigger_volume_->setWorldTransform(start_transform);

  physics_manager_ =
      &GetView().GetRegistry().GetOrCreate<PhysicsManager>(GetView());
  physics_manager_->AddTriggerVolume(*trigger_volume_, GetNode());

#if IMP_RUNTIME(DEV)
  physics_manager_->RegisterDebugVisualizer(
      GetNode(), [this]() { Visualize(); }, kCollidableVisualizer);
#endif

  return absl::OkStatus();
}

void TriggerVolume::Cleanup() {
  if (trigger_volume_) {
    physics_manager_->RemoveTriggerVolume(*trigger_volume_);
  }

#if IMP_RUNTIME(DEV)
  physics_manager_->UnRegisterDebugVisualizer(GetNode(), kCollidableVisualizer);
#endif
}

void TriggerVolume::Update(const FrameTime& frame_time) {
  // Update Bullet collider's transformation.
  trigger_volume_->setWorldTransform(collidable_.GetNodeBtTransform());
}

Collidable::CollisionShape TriggerVolume::GetCollisionShape() const {
  return collidable_.GetCollisionShape(trigger_volume_->getWorldTransform());
}

#if IMP_RUNTIME(DEV)
void TriggerVolume::Visualize() {
  if (!trigger_volume_) {
    return;
  }

  collidable_.Visualize(collidable_.GetNodeBtTransform());
}
#endif

}  // namespace imp
