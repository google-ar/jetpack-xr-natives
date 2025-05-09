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

#include "core/view/framework/collision/collision_manager.h"

#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/variant.h"
#include "core/collision/collision_flags.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/collision_system.h"
#include "core/view/framework/collision/mesh_collider.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/collision/sphere_collider.h"

namespace imp {
CollisionManager::CollisionManager(BaseView* view)
    : view_(view),
      collision_mask_(CollisionMask::kDefault),
      collision_mask_marker_(1 << kNumExistingCollisionMask) {
  AddCollisionSystem<BoxCollider, CollisionSystem>(view);
  AddCollisionSystem<SphereCollider, CollisionSystem>(view);
  AddCollisionSystem<GltfCollider, CollisionSystem>(view);
  AddCollisionSystem<MeshCollider, CollisionSystem>(view);
}

std::vector<RayHit> CollisionManager::IntersectAll(
    const Ray& world_ray, std::optional<Flags<CollisionMask>> mask) {
  return IntersectAllHelper(world_ray, mask);
}

std::vector<RayHit> CollisionManager::IntersectAll(
    float2 screen_pos, std::optional<Flags<CollisionMask>> mask) {
  return IntersectAllHelper(screen_pos, mask);
}

std::vector<DoubleRayHit> CollisionManager::IntersectAllPrecise(
    const DoubleRay& world_ray, std::optional<Flags<CollisionMask>> mask) {
  return IntersectAllPreciseHelper(world_ray, mask);
}

std::vector<DoubleRayHit> CollisionManager::IntersectAllPrecise(
    float2 screen_pos, std::optional<Flags<CollisionMask>> mask) {
  return IntersectAllPreciseHelper(screen_pos, mask);
}

std::vector<RayHit> CollisionManager::IntersectNode(
    NodeHandle node, const Ray& world_ray,
    std::optional<Flags<CollisionMask>> mask) {
  return IntersectNodeHelper(node, world_ray, mask);
}

std::vector<RayHit> CollisionManager::IntersectNode(
    NodeHandle node, float2 screen_pos,
    std::optional<Flags<CollisionMask>> mask) {
  return IntersectNodeHelper(node, screen_pos, mask);
}

template <typename T>
std::vector<RayHit> CollisionManager::IntersectNodeHelper(
    NodeHandle node, const T& world_ray_or_screen_pos,
    std::optional<Flags<CollisionMask>> mask) {
  if (!mask.has_value()) {
    mask = collision_mask_;
  }
  std::vector<RayHit> intersections;
  // Run against all types.
  for (auto& pair : collision_systems_) {
    pair.second->IntersectNode(node, world_ray_or_screen_pos, *mask,
                               &intersections);
  }
  SortFrontToBack(&intersections);
  return intersections;
}

std::vector<DoubleRayHit> CollisionManager::IntersectNodePrecise(
    NodeHandle node, const DoubleRay& world_ray,
    std::optional<Flags<CollisionMask>> mask) {
  return IntersectNodePreciseHelper(node, world_ray, mask);
}

std::vector<DoubleRayHit> CollisionManager::IntersectNodePrecise(
    NodeHandle node, float2 screen_pos,
    std::optional<Flags<CollisionMask>> mask) {
  return IntersectNodePreciseHelper(node, screen_pos, mask);
}

template <typename T>
std::vector<DoubleRayHit> CollisionManager::IntersectNodePreciseHelper(
    NodeHandle node, const T& world_ray_or_screen_pos,
    std::optional<Flags<CollisionMask>> mask) {
  if (!mask.has_value()) {
    mask = collision_mask_;
  }
  std::vector<DoubleRayHit> intersections;
  // Run against all types.
  for (auto& pair : collision_systems_) {
    pair.second->IntersectNodePrecise(node, world_ray_or_screen_pos, *mask,
                                      &intersections);
  }
  SortFrontToBack(&intersections);
  return intersections;
}

std::vector<RayHit> CollisionManager::IntersectAllHelper(
    absl::variant<Ray, float2> world_ray_or_screen_pos,
    std::optional<Flags<CollisionMask>> mask) {
  if (!mask.has_value()) {
    mask = collision_mask_;
  }
  std::vector<RayHit> intersections;
  // Tests against all types.
  for (auto& pair : collision_systems_) {
    if (absl::holds_alternative<float2>(world_ray_or_screen_pos)) {
      pair.second->Intersect(absl::get<float2>(world_ray_or_screen_pos), *mask,
                             &intersections);
    } else {
      pair.second->Intersect(absl::get<Ray>(world_ray_or_screen_pos), *mask,
                             &intersections);
    }
  }
  // Sorts and return results.
  SortFrontToBack(&intersections);
  return intersections;
}

std::vector<DoubleRayHit> CollisionManager::IntersectAllPreciseHelper(
    absl::variant<DoubleRay, float2> world_ray_or_screen_pos,
    std::optional<Flags<CollisionMask>> mask) {
  if (!mask.has_value()) {
    mask = collision_mask_;
  }
  std::vector<DoubleRayHit> intersections;
  // Tests against all types.
  for (auto& pair : collision_systems_) {
    if (absl::holds_alternative<float2>(world_ray_or_screen_pos)) {
      pair.second->IntersectPrecise(absl::get<float2>(world_ray_or_screen_pos),
                                    *mask, &intersections);
    } else {
      pair.second->IntersectPrecise(
          absl::get<DoubleRay>(world_ray_or_screen_pos), *mask, &intersections);
    }
  }
  // Sorts and return results.
  SortFrontToBack(&intersections);
  return intersections;
}

absl::StatusOr<Flags<CollisionMask>>
CollisionManager::CreateNewCollisionMask() {
  if (collision_mask_marker_ == 1 << (kCollisionMaskCountMax - 1)) {
    return absl::InternalError("All available masks are used.");
  }

  collision_mask_marker_ <<= 1;
  return ToFlags(static_cast<CollisionMask>(collision_mask_marker_));
}

#if IMP_RUNTIME(DEV)
void CollisionManager::VisualizeCollidersForNode(
    NodeHandle node, VisualizationStyle visualization_style) {
  // Run against all types.
  for (auto& pair : collision_systems_) {
    pair.second->VisualizeCollidersForNode(node, visualization_style);
  }
}
#endif

bool CollisionManager::HasCollider(NodeHandle node) const {
  for (auto& [component_id, _] : collision_systems_) {
    if (auto component_pool =
            view_->GetComponentManager().GetComponentPoolById(component_id);
        component_pool && component_pool->Has(node.GetEntity())) {
      return true;
    }
  }
  return false;
}

}  // namespace imp
