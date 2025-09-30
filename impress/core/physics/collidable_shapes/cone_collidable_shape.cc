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

#include "core/physics/collidable_shapes/cone_collidable_shape.h"

#include <memory>

#include "absl/log/check.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/BulletCollision/CollisionShapes/btConeShape.h"
#include "bullet/src/LinearMath/btScalar.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "bullet/src/LinearMath/btVector3.h"
#include "core/config.h"
#include "core/geometry/shapes/cone.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/collidable_shapes/collidable_shape.h"
#include "core/physics/physics_helper.h"
#include "core/view/framework/collision/cone_collider.h"
#if IMP_RUNTIME(DEV)
#include "core/common/debug_draw.h"
#endif

namespace imp {
ConeCollidableShape::ConeCollidableShape(NodeHandle node)
    : CollidableShape(node) {}

void ConeCollidableShape::CreateBtCollisionShape() {
  auto collider = node_->GetComponent<ConeCollider>();
  Cone local_cone = collider->GetCone();
  btScalar radius = btScalar(local_cone.radius);
  btScalar height = btScalar(local_cone.height);
  collidable_shape_ = std::make_unique<btConeShape>(radius, height);

  // In bullet, "most primitive shapes are centered around the origin of their
  // local coordinate frame", that is half way between the base and the tip of
  // the cone.
  collidable_center_ = local_cone.base + local_cone.height / 2 * kUp;
}

btCollisionShape* ConeCollidableShape::GetBtCollisionShape() const {
  return collidable_shape_.get();
}

float3 ConeCollidableShape::GetCollidableCenter() const {
  return collidable_center_;
}

CollidableShape::CollisionShape ConeCollidableShape::GetCollisionShape(
    const btTransform& bt_trans) const {
  const btConeShape* cone = static_cast<btConeShape*>(collidable_shape_.get());
  

  float radius = cone->getRadius();
  float height = cone->getHeight();
  btVector3 base = bt_trans * ToBtVector3(-height / 2 * kUp);

  return Cone(ToFloat3(base), radius, height);
}

void ConeCollidableShape::ApplyScalingToBulletCollider() {
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
void ConeCollidableShape::Visualize(const btTransform& bt_trans) const {
  const float3 world_scale = node_->GetWorldScale();
  if (world_scale.x > 0 && world_scale.y > 0 && world_scale.z > 0) {
    btConeShape* cone = static_cast<btConeShape*>(collidable_shape_.get());
    

    const float3 local_center =
        node_->LocalFromWorldPoint(ToVec3<float>(bt_trans.getOrigin()));
    float height = cone->getHeight() / world_scale.y;

    float3 base = local_center - height / 2 * kUp;
    float radius = cone->getRadius() / world_scale.x;

    debug_draw::Local(node_.GetEntity())
        .ConeLines(base, radius, height,
                   debug_draw::GetColor(debug_draw::DebugColor::kDeepOrange));
  }
}
#endif

}  // namespace imp
