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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COMPOUND_COLLIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COMPOUND_COLLIDER_H_

#include "absl/types/optional.h"
#include "core/collision/collider_mask_helpers.h"
#include "core/collision/collision_flags.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/geometry/shapes/compound_shape.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/capsule_collider.h"
#include "core/view/framework/collision/collider_state.proto.imp.h"
#include "core/view/framework/collision/cone_collider.h"
#include "core/view/framework/collision/cylinder_collider.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/collision/sphere_collider.h"

namespace imp {

// Enables compound shape collision testing on a Node.
class CompoundCollider : public Component,
                         public ColliderMaskHelpers<CompoundCollider> {
 public:
  void Setup();
  void Setup(const CompoundShape& compound_shape);
  void SetupWithState();

  void Cleanup();

  void SetCompoundShape(const CompoundShape& compound_shape);
  CompoundShape GetCompoundShape() const;

  // This is a pseudo-collider that does not register itself with the collision
  // system and lets its children handle visualization and collision testing.

  // Tests this node against a ray.
  // Optionally returns RayHit collision info if a collision occurred.
  // N.B. The Intersect methods are not used by the collision system, but are
  // provided only for convenience to the user.
  absl::optional<RayHit> Intersect(const Ray& ray);
  absl::optional<DoubleRayHit> IntersectPrecise(const DoubleRay& ray);

  void OnActiveStatusChanged(bool is_active);

 private:
  friend class ColliderMaskHelpers<CompoundCollider>;
  CompoundColliderState state_;
  CompoundShape compound_shape_;
  Flags<CollisionMask> collision_flags_{CollisionMask::kDefault};
  bool should_update_children_hit_node_ = true;

 public:
  using IsfInfo =
      IsfInfo<&CompoundCollider::state_,
              IsfDependencies<SphereCollider, BoxCollider, CapsuleCollider,
                              CylinderCollider, ConeCollider>>;
  static constexpr bool kRunInEditMode = true;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COMPOUND_COLLIDER_H_
