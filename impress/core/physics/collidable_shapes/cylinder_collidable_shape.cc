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

#include "core/physics/collidable_shapes/cylinder_collidable_shape.h"

#include <memory>

#include "absl/log/check.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCylinderShape.h"
#include "bullet/src/LinearMath/btScalar.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "bullet/src/LinearMath/btVector3.h"
#include "core/config.h"
#include "core/geometry/shapes/cylinder.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/collidable_shapes/collidable_shape.h"
#include "core/physics/physics_helper.h"
#include "core/view/framework/collision/cylinder_collider.h"
#if IMP_RUNTIME(DEV)
#include "core/common/debug_draw.h"
#endif

namespace imp {
CylinderCollidableShape::CylinderCollidableShape(NodeHandle node)
    : CollidableShape(node) {}

void CylinderCollidableShape::CreateBtCollisionShape() {
  auto collider = node_->GetComponent<CylinderCollider>();
  Cylinder local_cylinder = collider->GetCylinder();
  btVector3 half_extents(btScalar(local_cylinder.radius),
                         btScalar(local_cylinder.height / 2), {});
  collidable_shape_ = std::make_unique<btCylinderShape>(half_extents);

  // In bullet, "most primitive shapes are centered around the origin of their
  // local coordinate frame", that is half way between the two ends of the
  // cylinder.
  collidable_center_ = local_cylinder.base + local_cylinder.height / 2 * kUp;
}

btCollisionShape* CylinderCollidableShape::GetBtCollisionShape() const {
  return collidable_shape_.get();
}

float3 CylinderCollidableShape::GetCollidableCenter() const {
  return collidable_center_;
}

CollidableShape::CollisionShape CylinderCollidableShape::GetCollisionShape(
    const btTransform& bt_trans) const {
  const btCylinderShape* cylinder =
      static_cast<btCylinderShape*>(collidable_shape_.get());
  

  float half_length = cylinder->getHalfExtentsWithMargin().getY();
  btVector3 base = bt_trans * ToBtVector3(-half_length * kUp);

  return Cylinder(ToFloat3(base), cylinder->getRadius(), half_length * 2);
}

void CylinderCollidableShape::ApplyScalingToBulletCollider() {
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
void CylinderCollidableShape::Visualize(const btTransform& bt_trans) const {
  const float3 world_scale = node_->GetWorldScale();
  if (world_scale.x > 0 && world_scale.y > 0 && world_scale.z > 0) {
    btCylinderShape* cylinder =
        static_cast<btCylinderShape*>(collidable_shape_.get());
    

    const float3 local_center =
        node_->LocalFromWorldPoint(ToVec3<float>(bt_trans.getOrigin()));
    float half_length =
        cylinder->getHalfExtentsWithMargin().getY() / world_scale.y;

    float3 base = local_center - half_length * kUp;
    float radius = cylinder->getRadius() / world_scale.x;
    float height = half_length * 2;

    debug_draw::Local(node_.GetEntity())
        .CylinderLines(
            base, radius, height,
            debug_draw::GetColor(debug_draw::DebugColor::kDeepOrange));
  }
}
#endif

}  // namespace imp
