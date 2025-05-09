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

#include "core/physics/collidable.h"

#include <memory>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "bullet/src/BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/physics/collidable_shapes/box_collidable_shape.h"
#include "core/physics/collidable_shapes/convex_hull_mesh_collidable_shape.h"
#include "core/physics/collidable_shapes/sphere_collidable_shape.h"
#include "core/physics/collidable_shapes/static_mesh_collidable_shape.h"
#include "core/physics/collidable_type.proto.imp.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/sphere_collider.h"

namespace imp {

// TODO: Add a test for this function.
absl::Status Collidable::Create(btTransform& start_transform, NodeHandle node,
                                physics::CollidableType collidable_type) {
  type_ = collidable_type;
  if (type_ == physics::CollidableType::AUTOMATIC_DEFAULT ||
      type_ == physics::CollidableType::PRIMITIVE) {
    if (auto collider = node->GetComponent<SphereCollider>()) {
      collidable_shape_ = std::make_unique<SphereCollidableShape>(node);
      type_ = physics::CollidableType::PRIMITIVE;
    } else if (auto collider = node->GetComponent<BoxCollider>()) {
      collidable_shape_ = std::make_unique<BoxCollidableShape>(node);
      type_ = physics::CollidableType::PRIMITIVE;
    } else {
      if (type_ == physics::CollidableType::PRIMITIVE) {
        return absl::FailedPreconditionError("No primitive collider found");
      }
    }
  }
  if (type_ == physics::CollidableType::AUTOMATIC_DEFAULT ||
      type_ == physics::CollidableType::CONVEX_HULL ||
      type_ == physics::CollidableType::TRIANGLE_MESH_STATIC_ONLY) {
    if (auto mesh = node->GetComponent<GltfMesh>()) {
      if (mesh->GetMeshData().empty()) {
        return absl::FailedPreconditionError("No mesh data loaded");
      }
      if (mesh->IsSkinned()) {
        return absl::FailedPreconditionError(
            "Cannot add physics collider for skinned mesh");
      }
      if (type_ == physics::CollidableType::TRIANGLE_MESH_STATIC_ONLY) {
        collidable_shape_ = std::make_unique<StaticMeshCollidableShape>(node);
        is_movable_ = false;
      } else {
        collidable_shape_ =
            std::make_unique<ConvexHullMeshCollidableShape>(node);
        type_ = physics::CollidableType::CONVEX_HULL;
      }
    } else {
      if (type_ == physics::CollidableType::AUTOMATIC_DEFAULT) {
        return absl::FailedPreconditionError("No known collider found");
      }
      return absl::FailedPreconditionError("No GltfMesh found");
    }
  }

  start_transform = collidable_shape_->AddBtCollisionShape();
  ApplyScalingToBulletCollider();

  return absl::OkStatus();
}

btCollisionShape* Collidable::GetCollidableShape() {
  return collidable_shape_->GetCollidableShape();
}

float3 Collidable::GetCollidableCenter() {
  return collidable_shape_->GetCollidableCenter();
}

Collidable::CollisionShape Collidable::GetCollisionShape(
    const btTransform& transform) const {
  return collidable_shape_->GetCollisionShape(transform);
}

#if IMP_RUNTIME(DEV)
void Collidable::Visualize(const btTransform& transform) const {
  collidable_shape_->Visualize(transform);
}
#endif

void Collidable::ApplyScalingToBulletCollider() {
  // TODO: Need to fix: Scaling through transform widget fails
  // occasionally.
  collidable_shape_->ApplyScalingToBulletCollider();
}

}  // namespace imp
