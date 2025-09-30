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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_CYLINDER_COLLIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_CYLINDER_COLLIDER_H_

#include "absl/types/optional.h"
#include "core/collision/collider_mask_helpers.h"
#include "core/collision/collision_flags.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/geometry/shapes/cylinder.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/collision/collider_state.proto.imp.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

// Enables box collision testing on a Node.
class CylinderCollider : public Component,
                         public ColliderMaskHelpers<CylinderCollider> {
 public:
  void Setup();
  void Setup(const Cylinder& cylinder);
  void SetupWithState();

  void Cleanup();

  void SetCylinder(const Cylinder& cylinder);
  void SetCylinder(float3 base, float radius, float height);
  Cylinder GetCylinder() const;

  void Visualize(VisualizationStyle visualization_style =
                     VisualizationStyle::kNotSelected) const;

  // Tests this node against a ray.
  // Optionally returns RayHit collision info if a collision occurred.
  absl::optional<RayHit> Intersect(const Ray& ray);
  absl::optional<DoubleRayHit> IntersectPrecise(const DoubleRay& ray);

  // Sets the node that should be considered the "hit" node when this collider
  // collides with a ray.
  void SetHitNode(NodeHandle hit_node);
  NodeHandle GetHitNode() const;

  void OnActiveStatusChanged(bool is_active);

  // ComponentSystem for registering the CylinderCollider to the relevant
  // CollisionSystem
  class System : public ComponentSystem<CylinderCollider> {
   public:
    explicit System(BaseView* view) : ComponentSystem<CylinderCollider>(view) {}

    void BeforeFirstComponentAdded() override;
  };

 private:
  friend class ColliderMaskHelpers<CylinderCollider>;
  CylinderColliderState state_;
  Flags<CollisionMask> collision_flags_{CollisionMask::kDefault};
  NodeHandle hit_node_;

 public:
  using IsfInfo = IsfInfo<&CylinderCollider::state_>;
  static constexpr bool kRunInEditMode = true;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_CYLINDER_COLLIDER_H_
