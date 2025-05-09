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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COLLISION_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COLLISION_MANAGER_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/types/variant.h"
#include "core/collision/collision_flags.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/common/robin_map.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/collision/base_collision_system.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

class BaseView;

// CollisionManager manages collision systems and aggregates collision results.
class CollisionManager {
 public:
  explicit CollisionManager(BaseView* view);

  // Adds a collision system to the manager. The collider type, collision system
  // type, and the collision system constructor args must be provided.
  template <typename ColliderType, template <typename> typename System,
            typename... Args>
  CollisionManager& AddCollisionSystem(Args&&... args);

  // Removes the CollisionManager for the given <collider type, collision system
  // type> pair if one was previously added.
  template <typename ColliderType, template <typename> typename System>
  CollisionManager& RemoveCollisionSystem();

  // Tests ray against all registered collision systems.
  std::vector<RayHit> IntersectAll(
      const Ray& world_ray,
      std::optional<Flags<CollisionMask>> mask = std::nullopt);

  // Tests screen ray against all registered collision systems.
  std::vector<RayHit> IntersectAll(
      float2 screen_pos,
      std::optional<Flags<CollisionMask>> mask = std::nullopt);

  // Tests ray against all registered collision systems.
  std::vector<DoubleRayHit> IntersectAllPrecise(
      const DoubleRay& world_ray,
      std::optional<Flags<CollisionMask>> mask = std::nullopt);

  // Tests screen ray against all registered collision systems.
  std::vector<DoubleRayHit> IntersectAllPrecise(
      float2 screen_pos,
      std::optional<Flags<CollisionMask>> mask = std::nullopt);

  // Tests ray against one or more collider type collections, accepts an
  // optional mask to be tested against the selected collider types.
  template <typename... ColliderTypes>
  std::vector<RayHit> Intersect(
      const Ray& world_ray,
      Flags<CollisionMask> mask = Flags<CollisionMask>(CollisionMask::kAll));
  template <typename... ColliderTypes>
  std::vector<RayHit> Intersect(
      float2 screen_pos,
      Flags<CollisionMask> mask = Flags<CollisionMask>(CollisionMask::kAll));
  // Tests double ray against one or more collider type collections, accepts an
  // optional mask to be tested against the selected collider types.
  template <typename... ColliderTypes>
  std::vector<DoubleRayHit> IntersectPrecise(
      const DoubleRay& world_ray,
      Flags<CollisionMask> mask = Flags<CollisionMask>(CollisionMask::kAll));
  template <typename... ColliderTypes>
  std::vector<DoubleRayHit> IntersectPrecise(
      float2 screen_pos,
      Flags<CollisionMask> mask = Flags<CollisionMask>(CollisionMask::kAll));

  // Tests ray against a specific node for all registered collision systems.
  std::vector<RayHit> IntersectNode(
      NodeHandle node, const Ray& world_ray,
      std::optional<Flags<CollisionMask>> mask = std::nullopt);
  std::vector<RayHit> IntersectNode(
      NodeHandle node, float2 screen_pos,
      std::optional<Flags<CollisionMask>> mask = std::nullopt);
  std::vector<DoubleRayHit> IntersectNodePrecise(
      NodeHandle node, const DoubleRay& world_ray,
      std::optional<Flags<CollisionMask>> mask = std::nullopt);
  std::vector<DoubleRayHit> IntersectNodePrecise(
      NodeHandle node, float2 screen_pos,
      std::optional<Flags<CollisionMask>> mask = std::nullopt);

  // Returns true if the specified node has any collider with a registered
  // collision system.
  bool HasCollider(NodeHandle node) const;

  template <typename... ColliderTypes>
  size_t GetColliderCount();

  template <typename ColliderType>
  BaseCollisionSystem* GetCollisionSystem();

  Flags<CollisionMask> GetCollisionMask() const { return collision_mask_; }
  void SetCollisionMask(CollisionMask mask) { collision_mask_ = ToFlags(mask); }
  void SetCollisionMask(Flags<CollisionMask> mask) { collision_mask_ = mask; }
  absl::StatusOr<Flags<CollisionMask>> CreateNewCollisionMask();

#if IMP_RUNTIME(DEV)
  void VisualizeCollidersForNode(NodeHandle node,
                                 VisualizationStyle visualization_style);
#endif

 private:
  // Helper struct to contain just the type.
  template <typename T>
  struct TypeContainer {
    using type = T;
  };
  // Helper method to test one or more collider types against a ray.
  template <typename TUPLE, size_t IDX>
  constexpr void IntersectForEachColliderType(
      TUPLE collider_tuple, const Ray& world_ray, Flags<CollisionMask> mask,
      std::vector<RayHit>* out_intersections);
  // Helper method to test one or more collider types against a double ray.
  template <typename TUPLE, size_t IDX>
  constexpr void IntersectPreciseForEachColliderType(
      TUPLE collider_tuple, const DoubleRay& world_ray,
      Flags<CollisionMask> mask, std::vector<DoubleRayHit>* out_intersections);
  // Helper method to test one or more collider types against a ray.
  template <typename TUPLE, size_t IDX>
  constexpr void IntersectScreenForEachColliderType(
      TUPLE collider_tuple, float2 screen_pos, Flags<CollisionMask> mask,
      std::vector<RayHit>* out_intersections);
  // Helper method to test one or more collider types against a double ray.
  template <typename TUPLE, size_t IDX>
  constexpr void IntersectPreciseScreenForEachColliderType(
      TUPLE collider_tuple, float2 screen_pos, Flags<CollisionMask> mask,
      std::vector<DoubleRayHit>* out_intersections);

  template <typename TUPLE, size_t IDX>
  constexpr size_t GetCountForEachColliderType(TUPLE collider_tuple);

  template <typename T>
  void SortFrontToBack(std::vector<GenericRayHit<T>>* out_intersections);

  std::vector<RayHit> IntersectAllHelper(
      absl::variant<Ray, float2> world_ray_or_screen_pos,
      std::optional<Flags<CollisionMask>> mask);

  std::vector<DoubleRayHit> IntersectAllPreciseHelper(
      absl::variant<DoubleRay, float2> world_ray_or_screen_pos,
      std::optional<Flags<CollisionMask>> mask);

  template <typename T>
  std::vector<RayHit> IntersectNodeHelper(
      NodeHandle node, const T& world_ray_or_screen_pos,
      std::optional<Flags<CollisionMask>> mask);

  template <typename T>
  std::vector<DoubleRayHit> IntersectNodePreciseHelper(
      NodeHandle node, const T& world_ray_or_screen_pos,
      std::optional<Flags<CollisionMask>> mask);

  BaseView* view_;
  RobinMap<ComponentId, std::unique_ptr<BaseCollisionSystem>>
      collision_systems_;
  Flags<CollisionMask> collision_mask_;
  // Last used collision mask.
  uint32_t collision_mask_marker_;
};

template <typename ColliderType, template <typename> typename System,
          typename... Args>
CollisionManager& CollisionManager::AddCollisionSystem(Args&&... args) {
  ComponentId collider_type_hash = kComponentId<ColliderType>;
  if (collision_systems_.find(collider_type_hash) != collision_systems_.end()) {
    return *this;
  }
  collision_systems_.insert(
      {collider_type_hash,
       std::make_unique<System<ColliderType>>(std::forward<Args>(args)...)});
  return *this;
}

template <typename ColliderType, template <typename> typename System>
CollisionManager& CollisionManager::RemoveCollisionSystem() {
  ComponentId collider_type_hash = kComponentId<ColliderType>;
  collision_systems_.erase(collider_type_hash);
  return *this;
}

template <typename... ColliderTypes>
std::vector<RayHit> CollisionManager::Intersect(const Ray& world_ray,
                                                Flags<CollisionMask> mask) {
  std::vector<RayHit> intersections;
  // Tests each type provided.
  std::tuple<TypeContainer<ColliderTypes>...> colliders;
  IntersectForEachColliderType<decltype(colliders), 0>(colliders, world_ray,
                                                       mask, &intersections);
  // Sorts and return results.
  SortFrontToBack(&intersections);
  return intersections;
}

template <typename... ColliderTypes>
std::vector<DoubleRayHit> CollisionManager::IntersectPrecise(
    const DoubleRay& world_ray, Flags<CollisionMask> mask) {
  std::vector<DoubleRayHit> intersections;
  // Tests each type provided.
  std::tuple<TypeContainer<ColliderTypes>...> colliders;
  IntersectPreciseForEachColliderType<decltype(colliders), 0>(
      colliders, world_ray, mask, &intersections);
  // Sorts and return results.
  SortFrontToBack(&intersections);
  return intersections;
}

template <typename... ColliderTypes>
std::vector<RayHit> CollisionManager::Intersect(float2 screen_pos,
                                                Flags<CollisionMask> mask) {
  std::vector<RayHit> intersections;
  // Tests each type provided.
  std::tuple<TypeContainer<ColliderTypes>...> colliders;
  IntersectScreenForEachColliderType<decltype(colliders), 0>(
      colliders, screen_pos, mask, &intersections);
  // Sorts and return results.
  SortFrontToBack(&intersections);
  return intersections;
}

template <typename... ColliderTypes>
std::vector<DoubleRayHit> CollisionManager::IntersectPrecise(
    float2 screen_pos, Flags<CollisionMask> mask) {
  std::vector<DoubleRayHit> intersections;
  // Tests each type provided.
  std::tuple<TypeContainer<ColliderTypes>...> colliders;
  IntersectPreciseScreenForEachColliderType<decltype(colliders), 0>(
      colliders, screen_pos, mask, &intersections);
  // Sorts and return results.
  SortFrontToBack(&intersections);
  return intersections;
}

template <typename... ColliderTypes>
size_t CollisionManager::GetColliderCount() {
  std::tuple<TypeContainer<ColliderTypes>...> colliders;
  return GetCountForEachColliderType<decltype(colliders), 0>(colliders);
}

template <typename ColliderType>
BaseCollisionSystem* CollisionManager::GetCollisionSystem() {
  auto iter = collision_systems_.find(kComponentId<ColliderType>);
  if (iter != collision_systems_.end()) {
    return iter->second.get();
  }
  return nullptr;
}

template <typename TUPLE, size_t IDX>
constexpr void CollisionManager::IntersectForEachColliderType(
    TUPLE collider_tuple, const Ray& world_ray, Flags<CollisionMask> mask,
    std::vector<RayHit>* out_intersections) {
  // Test colliders of type 'collider_type' against the ray.
  using collider_type = typename std::tuple_element<IDX, TUPLE>::type::type;
  auto iter = collision_systems_.find(kComponentId<collider_type>);
  if (iter != collision_systems_.end()) {
    iter->second->Intersect(world_ray, mask, out_intersections);
  }

  // Recurse to the next collider type in the tuple.
  constexpr size_t kNext = IDX + 1;
  constexpr size_t kMax = std::tuple_size<TUPLE>::value;
  if constexpr (kNext < kMax) {
    IntersectForEachColliderType<TUPLE, std::min(kNext, kMax)>(
        collider_tuple, world_ray, mask, out_intersections);
  }
}

template <typename TUPLE, size_t IDX>
constexpr void CollisionManager::IntersectPreciseForEachColliderType(
    TUPLE collider_tuple, const DoubleRay& world_ray, Flags<CollisionMask> mask,
    std::vector<DoubleRayHit>* out_intersections) {
  // Test colliders of type 'collider_type' against the ray.
  using collider_type = typename std::tuple_element<IDX, TUPLE>::type::type;
  auto iter = collision_systems_.find(kComponentId<collider_type>);
  if (iter != collision_systems_.end()) {
    iter->second->IntersectPrecise(world_ray, mask, out_intersections);
  }

  // Recurse to the next collider type in the tuple.
  constexpr size_t kNext = IDX + 1;
  constexpr size_t kMax = std::tuple_size<TUPLE>::value;
  if constexpr (kNext < kMax) {
    IntersectPreciseForEachColliderType<TUPLE, std::min(kNext, kMax)>(
        collider_tuple, world_ray, mask, out_intersections);
  }
}

template <typename TUPLE, size_t IDX>
constexpr void CollisionManager::IntersectScreenForEachColliderType(
    TUPLE collider_tuple, float2 screen_pos, Flags<CollisionMask> mask,
    std::vector<RayHit>* out_intersections) {
  // Test colliders of type 'collider_type' against the ray.
  using collider_type = typename std::tuple_element<IDX, TUPLE>::type::type;
  auto iter = collision_systems_.find(kComponentId<collider_type>);
  if (iter != collision_systems_.end()) {
    iter->second->Intersect(screen_pos, mask, out_intersections);
  }

  // Recurse to the next collider type in the tuple.
  constexpr size_t kNext = IDX + 1;
  constexpr size_t kMax = std::tuple_size<TUPLE>::value;
  if constexpr (kNext < kMax) {
    IntersectScreenForEachColliderType<TUPLE, std::min(kNext, kMax)>(
        collider_tuple, screen_pos, mask, out_intersections);
  }
}

template <typename TUPLE, size_t IDX>
constexpr void CollisionManager::IntersectPreciseScreenForEachColliderType(
    TUPLE collider_tuple, float2 screen_pos, Flags<CollisionMask> mask,
    std::vector<DoubleRayHit>* out_intersections) {
  // Test colliders of type 'collider_type' against the ray.
  using collider_type = typename std::tuple_element<IDX, TUPLE>::type::type;
  auto iter = collision_systems_.find(kComponentId<collider_type>);
  if (iter != collision_systems_.end()) {
    iter->second->IntersectPrecise(screen_pos, mask, out_intersections);
  }

  // Recurse to the next collider type in the tuple.
  constexpr size_t kNext = IDX + 1;
  constexpr size_t kMax = std::tuple_size<TUPLE>::value;
  if constexpr (kNext < kMax) {
    IntersectPreciseScreenForEachColliderType<TUPLE, std::min(kNext, kMax)>(
        collider_tuple, screen_pos, mask, out_intersections);
  }
}

template <typename TUPLE, size_t IDX>
constexpr size_t CollisionManager::GetCountForEachColliderType(
    TUPLE collider_tuple) {
  // Test colliders of type 'collider_type' against the ray.
  using collider_type = typename std::tuple_element<IDX, TUPLE>::type::type;
  auto iter = collision_systems_.find(kComponentId<collider_type>);
  size_t result = 0;
  if (iter != collision_systems_.end()) {
    result += iter->second->GetColliderCount();
  }

  // Recurse to the next collider type in the tuple.
  constexpr size_t kNext = IDX + 1;
  constexpr size_t kMax = std::tuple_size<TUPLE>::value;
  if constexpr (kNext < kMax) {
    result += GetCountForEachColliderType<TUPLE, std::min(kNext, kMax)>(
        collider_tuple);
  }
  return result;
}

template <typename T>
void CollisionManager::SortFrontToBack(
    std::vector<GenericRayHit<T>>* out_intersections) {
  std::sort(out_intersections->begin(), out_intersections->end(),
            [](const GenericRayHit<T>& a, const GenericRayHit<T>& b) {
              return a.distance < b.distance;
            });
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COLLISION_MANAGER_H_
