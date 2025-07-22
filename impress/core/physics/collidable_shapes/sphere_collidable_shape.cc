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

#include "core/physics/collidable_shapes/sphere_collidable_shape.h"

#include <cmath>
#include <limits>
#include <memory>

#include "absl/log/check.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/BulletCollision/CollisionShapes/btSphereShape.h"
#include "bullet/src/LinearMath/btScalar.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/config.h"
#include "core/geometry/shapes/sphere.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/collidable_shapes/collidable_shape.h"
#include "core/physics/physics_helper.h"
#include "core/view/framework/collision/sphere_collider.h"
#if IMP_RUNTIME(DEV)
#include "core/common/debug_draw.h"
#endif

namespace imp {
SphereCollidableShape::SphereCollidableShape(NodeHandle node) : node_(node) {}

void SphereCollidableShape::CreateBtCollisionShape() {
  auto collider = node_->GetComponent<SphereCollider>();
  Sphere local_sphere = collider->GetSphere();
  collidable_shape_ =
      std::make_unique<btSphereShape>(btScalar(local_sphere.radius));

  collidable_center_ = local_sphere.center;
}

btCollisionShape* SphereCollidableShape::GetBtCollisionShape() const {
  return collidable_shape_.get();
}

float3 SphereCollidableShape::GetCollidableCenter() const {
  return collidable_center_;
}

CollidableShape::CollisionShape SphereCollidableShape::GetCollisionShape(
    const btTransform& bt_trans) const {
  const btSphereShape* sphere =
      static_cast<btSphereShape*>(collidable_shape_.get());
  
  return Sphere(ToVec3<float>(bt_trans.getOrigin()), sphere->getRadius());
}

void SphereCollidableShape::ApplyScalingToBulletCollider() {
  const float3 scale = node_->GetWorldScale();
  if (scale.x > 0 && scale.y > 0 && scale.z > 0) {
    EnforceEvenScaleForSphere();
    return;
  }
  scale_prev_ = scale;
}

#if IMP_RUNTIME(DEV)
void SphereCollidableShape::Visualize(const btTransform& bt_trans) const {
  const float3 world_scale = node_->GetWorldScale();
  if (world_scale.x > 0 && world_scale.y > 0 && world_scale.z > 0) {
    btSphereShape* sphere =
        static_cast<btSphereShape*>(collidable_shape_.get());
    

    const float3 local_center =
        node_->LocalFromWorldPoint(ToVec3<float>(bt_trans.getOrigin()));
    debug_draw::Local(node_.GetEntity())
        .SphereLines(local_center, sphere->getRadius() / world_scale.x,
                     debug_draw::GetColor(debug_draw::DebugColor::kDeepOrange));
  }
}
#endif

void SphereCollidableShape::EnforceEvenScaleForSphere() {
  const float3 sphere_scale = node_->GetWorldScale();
  const bool x_changed = !RoughlyEqual(sphere_scale.x, scale_prev_.x);
  const bool y_changed = !RoughlyEqual(sphere_scale.y, scale_prev_.y);
  const bool z_changed = !RoughlyEqual(sphere_scale.z, scale_prev_.z);

  if (!x_changed && !y_changed && !z_changed) {
    return;
  }
  float new_scale = std::numeric_limits<float>::min();
  if (x_changed) {
    new_scale = fmax(sphere_scale.x, new_scale);
  }
  if (y_changed) {
    new_scale = fmax(sphere_scale.y, new_scale);
  }
  if (z_changed) {
    new_scale = fmax(sphere_scale.z, new_scale);
  }
  node_->SetWorldScale(new_scale);
  collidable_shape_->setLocalScaling(ToBtVector3(new_scale));
  scale_prev_ = new_scale;
}

}  // namespace imp
