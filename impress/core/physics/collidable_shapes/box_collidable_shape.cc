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

#include "core/physics/collidable_shapes/box_collidable_shape.h"

#include <memory>

#include "absl/log/check.h"
#include "bullet/src/BulletCollision/CollisionShapes/btBoxShape.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/config.h"
#include "core/geometry/shapes/box.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/collidable_shapes/collidable_shape.h"
#include "core/physics/physics_helper.h"
#include "core/view/framework/collision/box_collider.h"
#if IMP_RUNTIME(DEV)
#include "core/common/debug_draw.h"
#endif

namespace imp {
BoxCollidableShape::BoxCollidableShape(NodeHandle node) : node_(node) {}

void BoxCollidableShape::CreateBtCollisionShape() {
  auto collider = node_->GetComponent<BoxCollider>();
  Box local_box = collider->GetLocalBox();
  collidable_shape_ =
      std::make_unique<btBoxShape>(ToBtVector3(local_box.halfExtent));

  collidable_center_ = local_box.center;
}

btCollisionShape* BoxCollidableShape::GetBtCollisionShape() const {
  return collidable_shape_.get();
}

float3 BoxCollidableShape::GetCollidableCenter() const {
  return collidable_center_;
}

CollidableShape::CollisionShape BoxCollidableShape::GetCollisionShape(
    const btTransform& bt_trans) const {
  const btBoxShape* box = static_cast<btBoxShape*>(collidable_shape_.get());
  
  return Box{.center = ToVec3<float>(bt_trans.getOrigin()),
             .halfExtent = ToVec3<float>(box->getHalfExtentsWithMargin())};
}

void BoxCollidableShape::ApplyScalingToBulletCollider() {
  const float3 scale = node_->GetWorldScale();
  if (scale.x > 0 && scale.y > 0 && scale.z > 0) {
    collidable_shape_->setLocalScaling(ToBtVector3(scale));
  }
  scale_prev_ = scale;
}

#if IMP_RUNTIME(DEV)
void BoxCollidableShape::Visualize(const btTransform& bt_trans) const {
  const float3 world_scale = node_->GetWorldScale();
  if (world_scale.x > 0 && world_scale.y > 0 && world_scale.z > 0) {
    btBoxShape* box = static_cast<btBoxShape*>(collidable_shape_.get());

    const float3 local_box_center =
        node_->LocalFromWorldPoint(ToVec3<float>(bt_trans.getOrigin()));
    float3 local_box_half_extent =
        ToVec3<float>(box->getHalfExtentsWithMargin());
    local_box_half_extent.x /= world_scale.x;
    local_box_half_extent.y /= world_scale.y;
    local_box_half_extent.z /= world_scale.z;

    debug_draw::Local(node_.GetEntity())
        .BoxLines(Box{.center = local_box_center,
                      .halfExtent = local_box_half_extent},
                  debug_draw::GetColor(debug_draw::DebugColor::kDeepOrange));
  }
}
#endif

}  // namespace imp
