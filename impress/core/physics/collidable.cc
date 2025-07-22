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
#include "core/physics/collidable_shapes/capsule_collidable_shape.h"
#include "core/physics/collidable_shapes/cone_collidable_shape.h"
#include "core/physics/collidable_shapes/convex_hull_mesh_collidable_shape.h"
#include "core/physics/collidable_shapes/cylinder_collidable_shape.h"
#include "core/physics/collidable_shapes/sphere_collidable_shape.h"
#include "core/physics/collidable_shapes/static_mesh_collidable_shape.h"
#include "core/physics/collidable_type.proto.imp.h"
#include "core/physics/physics_helper.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/capsule_collider.h"
#include "core/view/framework/collision/cone_collider.h"
#include "core/view/framework/collision/cylinder_collider.h"
#include "core/view/framework/collision/sphere_collider.h"
#include "core/view/framework/render/mesh_renderer.h"

namespace imp {

// TODO: Add a test for this function.
absl::Status Collidable::Create(btTransform& start_transform, NodeHandle node,
                                physics::CollidableType collidable_type) {
  node_ = node;
  type_ = collidable_type;
  if (type_ == physics::CollidableType::AUTOMATIC_DEFAULT ||
      type_ == physics::CollidableType::PRIMITIVE) {
    if (auto collider = node_->GetComponent<SphereCollider>()) {
      collidable_shape_ = std::make_unique<SphereCollidableShape>(node_);
      type_ = physics::CollidableType::PRIMITIVE;
    } else if (auto collider = node_->GetComponent<BoxCollider>()) {
      collidable_shape_ = std::make_unique<BoxCollidableShape>(node_);
      type_ = physics::CollidableType::PRIMITIVE;
    } else if (auto collider = node_->GetComponent<CylinderCollider>()) {
      collidable_shape_ = std::make_unique<CylinderCollidableShape>(node_);
      type_ = physics::CollidableType::PRIMITIVE;
    } else if (auto collider = node_->GetComponent<ConeCollider>()) {
      collidable_shape_ = std::make_unique<ConeCollidableShape>(node_);
      type_ = physics::CollidableType::PRIMITIVE;
    } else if (auto collider = node_->GetComponent<CapsuleCollider>()) {
      collidable_shape_ = std::make_unique<CapsuleCollidableShape>(node_);
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
    if (auto mesh = node_->GetComponent<GltfMesh>()) {
      if (mesh->GetMeshData().empty()) {
        return absl::FailedPreconditionError("No mesh data loaded");
      }
      if (mesh->IsSkinned()) {
        return absl::FailedPreconditionError(
            "Cannot add physics collider for skinned mesh");
      }
    } else if (auto mesh_renderer = node_->GetComponent<MeshRenderer>()) {
      if (mesh_renderer->GetPrimitiveCount() == 0) {
        return absl::FailedPreconditionError(
            "No mesh data loaded from MeshRenderer");
      }
    } else {
      if (type_ == physics::CollidableType::AUTOMATIC_DEFAULT) {
        return absl::FailedPreconditionError("No known collider found");
      }
      return absl::FailedPreconditionError("No GltfMesh/MeshRenderer found");
    }

    if (type_ == physics::CollidableType::TRIANGLE_MESH_STATIC_ONLY) {
      collidable_shape_ = std::make_unique<StaticMeshCollidableShape>(node_);
      is_movable_ = false;
    } else {
      collidable_shape_ =
          std::make_unique<ConvexHullMeshCollidableShape>(node_);
      type_ = physics::CollidableType::CONVEX_HULL;
    }
    collidable_shape_ = std::make_unique<StaticMeshCollidableShape>(node_);
  }

  collidable_shape_->CreateBtCollisionShape();

  start_transform = GetNodeBtTransform();

  return absl::OkStatus();
}

btCollisionShape* Collidable::GetBtCollisionShape() const {
  return collidable_shape_->GetBtCollisionShape();
}

float3 Collidable::GetCollidableCenter() const {
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

void Collidable::ApplyScalingToBulletCollider() const {
  // TODO: Need to fix: Scaling through transform widget fails
  // occasionally.
  collidable_shape_->ApplyScalingToBulletCollider();
}

btTransform Collidable::GetNodeBtTransform() const {
  ApplyScalingToBulletCollider();
  float3 center_offset =
      node_->GetWorldRotation() * collidable_shape_->GetCollidableCenter();
  return ToBtTransform(
      node_->GetWorldPosition() + center_offset * node_->GetWorldScale(),
      node_->GetWorldRotation());
}

}  // namespace imp
