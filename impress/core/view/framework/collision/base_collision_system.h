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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_BASE_COLLISION_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_BASE_COLLISION_SYSTEM_H_
#include <cstddef>
#include <vector>

#include "core/collision/collision_flags.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/system.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

class BaseView;

// Abstract base class/interface for all collision systems.
class BaseCollisionSystem : public System {
 public:
  explicit BaseCollisionSystem(BaseView* view) : System(view) {}

  // Tests all colliders against a ray.
  virtual void Intersect(const Ray& world_ray, Flags<CollisionMask> mask,
                         std::vector<RayHit>* out_intersections) = 0;

  // Optional collision test using a screen position.
  virtual void Intersect(float2 screen_pos, Flags<CollisionMask> mask,
                         std::vector<RayHit>* out_intersections) {}

  // Tests all colliders against a double ray.
  virtual void IntersectPrecise(
      const DoubleRay& world_ray, Flags<CollisionMask> mask,
      std::vector<DoubleRayHit>* out_intersections) = 0;

  // Optional collision test using a screen position.
  virtual void IntersectPrecise(float2 screen_pos, Flags<CollisionMask> mask,
                                std::vector<DoubleRayHit>* out_intersections) {}

  // Tests all colliders for a specific node against a ray.
  virtual void IntersectNode(NodeHandle node, const Ray& world_ray,
                             Flags<CollisionMask> mask,
                             std::vector<RayHit>* out_intersections) {}

  // Optional collision test using a screen position.
  virtual void IntersectNode(NodeHandle node, float2 screen_pos,
                             Flags<CollisionMask> mask,
                             std::vector<RayHit>* out_intersections) {}

  // Tests all colliders for a specific node against a double ray.
  virtual void IntersectNodePrecise(
      NodeHandle node, const DoubleRay& world_ray, Flags<CollisionMask> mask,
      std::vector<DoubleRayHit>* out_intersections) {}

  // Optional collision test using a screen position.
  virtual void IntersectNodePrecise(
      NodeHandle node, float2 screen_pos, Flags<CollisionMask> mask,
      std::vector<DoubleRayHit>* out_intersections) {}

  // Optional override for providing the number of colliders
  virtual size_t GetColliderCount() { return 0; }

#if IMP_RUNTIME(DEV)
  virtual void VisualizeCollidersForNode(
      NodeHandle node, VisualizationStyle visualization_style) {}
#endif
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_BASE_COLLISION_SYSTEM_H_
