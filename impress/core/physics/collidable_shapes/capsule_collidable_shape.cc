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

#include "core/physics/collidable_shapes/capsule_collidable_shape.h"

#include <memory>

#include "absl/log/check.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/LinearMath/btScalar.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/config.h"
#include "core/geometry/shapes/capsule.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/collidable_shapes/collidable_shape.h"
#include "core/physics/physics_helper.h"
#include "core/view/framework/collision/capsule_collider.h"
#if IMP_RUNTIME(DEV)
#include "core/common/debug_draw.h"
#endif

namespace imp {
CapsuleCollidableShape::CapsuleCollidableShape(NodeHandle node)
    : CollidableShape(node) {}

void CapsuleCollidableShape::CreateBtCollisionShape() {
  auto collider = node_->GetComponent<CapsuleCollider>();
  Capsule local_capsule = collider->GetCapsule();
  collidable_shape_ = std::make_unique<btCapsuleShape>(
      btScalar(local_capsule.radius), btScalar(local_capsule.height));

  collidable_center_ = local_capsule.center;
}

btCollisionShape* CapsuleCollidableShape::GetBtCollisionShape() const {
  return collidable_shape_.get();
}

float3 CapsuleCollidableShape::GetCollidableCenter() const {
  return collidable_center_;
}

CollidableShape::CollisionShape CapsuleCollidableShape::GetCollisionShape(
    const btTransform& bt_trans) const {
  const btCapsuleShape* capsule =
      static_cast<btCapsuleShape*>(collidable_shape_.get());
  
  return Capsule(ToVec3<float>(bt_trans.getOrigin()),
                 capsule->getHalfHeight() * 2, capsule->getRadius());
}

void CapsuleCollidableShape::ApplyScalingToBulletCollider() {
  const float3 scale = node_->GetWorldScale();
  if (scale.x > 0 && scale.y > 0 && scale.z > 0 &&
      UpdatedEvenScale(scale_, scale)) {
    node_->SetWorldScale(scale_);
    collidable_shape_->setLocalScaling(ToBtVector3(scale_));
    return;
  }
  scale_ = scale;
}

#if IMP_RUNTIME(DEV)
void CapsuleCollidableShape::Visualize(const btTransform& bt_trans) const {
  const float3 world_scale = node_->GetWorldScale();
  if (world_scale.x > 0 && world_scale.y > 0 && world_scale.z > 0) {
    btCapsuleShape* capsule =
        static_cast<btCapsuleShape*>(collidable_shape_.get());
    

    const float3 local_center =
        node_->LocalFromWorldPoint(ToVec3<float>(bt_trans.getOrigin()));
    debug_draw::Local(node_.GetEntity())
        .CapsuleLines(
            local_center, capsule->getHalfHeight() * 2 / world_scale.y,
            capsule->getRadius() / world_scale.x,
            debug_draw::GetColor(debug_draw::DebugColor::kDeepOrange));
  }
}
#endif

}  // namespace imp
