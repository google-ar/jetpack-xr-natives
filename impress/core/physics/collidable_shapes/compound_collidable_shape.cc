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

#include "core/physics/collidable_shapes/compound_collidable_shape.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCollisionShape.h"
#include "bullet/src/BulletCollision/CollisionShapes/btCompoundShape.h"
#include "bullet/src/LinearMath/btScalar.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "bullet/src/LinearMath/btVector3.h"
#include "core/config.h"
#include "core/geometry/shapes/compound_shape.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/physics/collidable_shapes/box_collidable_shape.h"
#include "core/physics/collidable_shapes/capsule_collidable_shape.h"
#include "core/physics/collidable_shapes/collidable_shape.h"
#include "core/physics/collidable_shapes/collidable_shape_properties.h"
#include "core/physics/collidable_shapes/cone_collidable_shape.h"
#include "core/physics/collidable_shapes/cylinder_collidable_shape.h"
#include "core/physics/collidable_shapes/sphere_collidable_shape.h"
#include "core/physics/physics_helper.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/capsule_collider.h"
#include "core/view/framework/collision/compound_collider.h"
#include "core/view/framework/collision/cone_collider.h"
#include "core/view/framework/collision/cylinder_collider.h"
#include "core/view/framework/collision/sphere_collider.h"
#if IMP_RUNTIME(DEV)
#include "core/common/debug_draw.h"
#endif

namespace imp {
CompoundCollidableShape::CompoundCollidableShape(NodeHandle node)
    : CollidableShape(node) {}

void CompoundCollidableShape::CreateBtCollisionShape() {
  auto collider = node_->GetComponent<CompoundCollider>();
  CompoundShape local_compound_shape = collider->GetCompoundShape();

  std::unique_ptr<btCompoundShape> compound_shape_temp =
      std::make_unique<btCompoundShape>();

  std::vector<btScalar> masses;

  for (auto& child : node_->GetView().GetPathManager().GetDescendants(node_)) {
    std::unique_ptr<CollidableShape> collidable_shape;
    auto lambda = [&]<class T>(auto collider) {
      collider->SetHitNode(node_);
      collidable_shape = std::make_unique<T>(child);
    };

    if (auto child_collider = child->GetComponent<SphereCollider>()) {
      lambda.template operator()<SphereCollidableShape>(child_collider);
    } else if (auto child_collider = child->GetComponent<BoxCollider>()) {
      lambda.template operator()<BoxCollidableShape>(child_collider);
    } else if (auto child_collider = child->GetComponent<CylinderCollider>()) {
      lambda.template operator()<CylinderCollidableShape>(child_collider);
    } else if (auto child_collider = child->GetComponent<ConeCollider>()) {
      lambda.template operator()<ConeCollidableShape>(child_collider);
    } else if (auto child_collider = child->GetComponent<CapsuleCollider>()) {
      lambda.template operator()<CapsuleCollidableShape>(child_collider);
    } else {
      continue;
    }

    collidable_shape->CreateBtCollisionShape();
    compound_shape_temp->addChildShape(collidable_shape->GetNodeBtTransform(),
                                       collidable_shape->GetBtCollisionShape());
    if (auto props = child->GetComponent<CollidableShapeProperties>()) {
      masses.push_back(props->GetState().mass);
    } else {
      masses.push_back(1);
    }
    child_shapes_.emplace_back(std::move(collidable_shape));
  }

  btTransform principal;
  btVector3 inertia;
  compound_shape_temp->calculatePrincipalAxisTransform(masses.data(), principal,
                                                       inertia);
  std::unique_ptr<btCompoundShape> compound_shape_aligned =
      std::make_unique<btCompoundShape>();

  // recompute the shift to make sure the compound shape is re-aligned
  btTransform principal_inv = principal.inverse();
  for (int i = 0; i < compound_shape_temp->getNumChildShapes(); i++) {
    compound_shape_aligned->addChildShape(
        principal_inv * compound_shape_temp->getChildTransform(i),
        compound_shape_temp->getChildShape(i));
  }

  collidable_shape_ = std::move(compound_shape_aligned);

  // The first number is the offset of the compound shape to its node center.
  // The second number is the distance from the computed center of mass to that
  // offset position.
  collidable_center_ =
      local_compound_shape.center + ToFloat3(principal.getOrigin());
}

btCollisionShape* CompoundCollidableShape::GetBtCollisionShape() const {
  return collidable_shape_.get();
}

float3 CompoundCollidableShape::GetCollidableCenter() const {
  return collidable_center_;
}

CollidableShape::CollisionShape CompoundCollidableShape::GetCollisionShape(
    const btTransform& bt_trans) const {
  const btCompoundShape* compound =
      static_cast<btCompoundShape*>(collidable_shape_.get());
  

  // TODO: Revisit for all CollidableShapes.
  btVector3 base = bt_trans * ToBtVector3(kZero3 - collidable_center_);

  return CompoundShape{ToFloat3(base)};
}

void CompoundCollidableShape::ApplyScalingToBulletCollider() {
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
void CompoundCollidableShape::Visualize(const btTransform& bt_trans) const {
  for (const auto& shape : child_shapes_) {
    shape->Visualize(shape->GetNodeBtTransform());
  }
}
#endif

}  // namespace imp
