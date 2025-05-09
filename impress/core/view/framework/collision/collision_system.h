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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COLLISION_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COLLISION_SYSTEM_H_

#include <vector>

#include "absl/types/optional.h"
#include "core/collision/collision_flags.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/common/platform_helpers.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"
#include "core/view/framework/collision/base_collision_system.h"
#include "core/view/framework/collision/collider_traits.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp {

namespace details {
Ray GetWorldRayFromPixelPosition(BaseView& view, float2 screen_pos);
DoubleRay GetWorldRayFromPixelPositionPrecise(BaseView& view,
                                              float2 screen_pos);
}  // namespace details

// The generic CollisionSystem provides enough functionality for most use cases.
template <typename T>
class CollisionSystem : public BaseCollisionSystem {
 public:
  explicit CollisionSystem(BaseView* view);

  // Tests colliders in the scene against a ray, applies collision mask filter
  // to colliders.
  void Intersect(const Ray& world_ray, Flags<CollisionMask> mask,
                 std::vector<RayHit>* out_intersections) override;

  void Intersect(float2 screen_pos, Flags<CollisionMask> mask,
                 std::vector<RayHit>* out_intersections) override;

  // Tests colliders in the scene against a double ray, applies collision mask
  // filter to colliders.
  void IntersectPrecise(const DoubleRay& world_ray, Flags<CollisionMask> mask,
                        std::vector<DoubleRayHit>* out_intersections) override;

  void IntersectPrecise(float2 screen_pos, Flags<CollisionMask> mask,
                        std::vector<DoubleRayHit>* out_intersections) override;

  void IntersectNode(NodeHandle node, const Ray& world_ray,
                     Flags<CollisionMask> mask,
                     std::vector<RayHit>* out_intersections) override;

  void IntersectNode(NodeHandle node, float2 screen_pos,
                     Flags<CollisionMask> mask,
                     std::vector<RayHit>* out_intersections) override;

  void IntersectNodePrecise(
      NodeHandle node, const DoubleRay& world_ray, Flags<CollisionMask> mask,
      std::vector<DoubleRayHit>* out_intersections) override;

  void IntersectNodePrecise(
      NodeHandle node, float2 screen_pos, Flags<CollisionMask> mask,
      std::vector<DoubleRayHit>* out_intersections) override;

#if IMP_RUNTIME(DEV)
  // Gets debug-geometry from colliders for use in the collider-visualizer
  // editor widget.
  void VisualizeCollidersForNode(
      NodeHandle node, VisualizationStyle visualization_style) override;
#endif

 protected:
  static void IntersectCollider(T* collider, const Ray& world_ray,
                                Flags<CollisionMask> mask,
                                std::vector<RayHit>* out_intersections);
  static void IntersectCollider(T* collider, const Ray& world_ray,
                                float2 screen_pos, Flags<CollisionMask> mask,
                                std::vector<RayHit>* out_intersections);
  static void IntersectColliderPrecise(
      T* collider, const DoubleRay& world_ray, Flags<CollisionMask> mask,
      std::vector<DoubleRayHit>* out_intersections);
  static void IntersectColliderPrecise(
      T* collider, const DoubleRay& world_ray, float2 screen_pos,
      Flags<CollisionMask> mask, std::vector<DoubleRayHit>* out_intersections);
};

template <typename T>
CollisionSystem<T>::CollisionSystem(BaseView* view)
    : BaseCollisionSystem(view) {
  static_assert(collider_traits::kHasIntersectWithRayFunc<T> ||
                    collider_traits::kHasMultipleIntersectWithRayFunc<T> ||
                    collider_traits::kHasIntersectWithScreenPosFunc<T> ||
                    collider_traits::kHasMultipleIntersectWithScreenPosFunc<T>,
                "Collider must define an Intersect method");
  static_assert(
      collider_traits::kHasIntersectPreciseWithDoubleRayFunc<T> ||
          collider_traits::kHasMultipleIntersectPreciseWithDoubleRayFunc<T> ||
          collider_traits::kHasIntersectPreciseWithScreenPosFunc<T> ||
          collider_traits::kHasMultipleIntersectPreciseWithScreenPosFunc<T>,
      "Collider must define an IntersectPrecise method");
}

template <typename T>
void CollisionSystem<T>::Intersect(const Ray& world_ray,
                                   Flags<CollisionMask> mask,
                                   std::vector<RayHit>* out_intersections) {
  GetComponentManager().template ForEach<T>(
      [&world_ray, &out_intersections, mask](T* collider) {
        IntersectCollider(collider, world_ray, mask, out_intersections);
      });
}

template <typename T>
void CollisionSystem<T>::IntersectCollider(
    T* collider, const Ray& world_ray, Flags<CollisionMask> mask,
    std::vector<RayHit>* out_intersections) {
  if constexpr (collider_traits::kHasIsActiveFunc<T>) {
    if (!collider->IsActive()) {
      return;
    }
  }

  if constexpr (collider_traits::kHasTestCollisionFlagsFunc<T>) {
    if (!collider->TestCollisionFlags(mask)) {
      return;
    }
  }
  if constexpr (collider_traits::kHasIntersectWithRayFunc<T>) {
    if (absl::optional<RayHit> result = collider->Intersect(world_ray)) {
      out_intersections->push_back(*result);
    }
  } else if constexpr (collider_traits::kHasMultipleIntersectWithRayFunc<T>) {
    std::vector<RayHit> result = collider->Intersect(world_ray);
    out_intersections->insert(out_intersections->end(), result.begin(),
                              result.end());
  }
}

template <typename T>
void CollisionSystem<T>::Intersect(float2 screen_pos, Flags<CollisionMask> mask,
                                   std::vector<RayHit>* out_intersections) {
  const Ray world_ray =
      details::GetWorldRayFromPixelPosition(GetView(), screen_pos);
  GetComponentManager().template ForEach<T>(
      [&screen_pos, &world_ray, &out_intersections, mask](T* collider) {
        IntersectCollider(collider, world_ray, screen_pos, mask,
                          out_intersections);
      });
}

template <typename T>
void CollisionSystem<T>::IntersectCollider(
    T* collider, const Ray& world_ray, float2 screen_pos,
    Flags<CollisionMask> mask, std::vector<RayHit>* out_intersections) {
  if constexpr (collider_traits::kHasIsActiveFunc<T>) {
    if (!collider->IsActive()) {
      return;
    }
  }

  if constexpr (collider_traits::kHasTestCollisionFlagsFunc<T>) {
    if (!collider->TestCollisionFlags(mask)) {
      return;
    }
  }
  if constexpr (collider_traits::kHasIntersectWithScreenPosFunc<T>) {
    if (absl::optional<RayHit> result = collider->Intersect(screen_pos)) {
      out_intersections->push_back(*result);
    }
  } else if constexpr (collider_traits::kHasMultipleIntersectWithScreenPosFunc<
                           T>) {
    std::vector<RayHit> result = collider->Intersect(screen_pos);
    out_intersections->insert(out_intersections->end(), result.begin(),
                              result.end());
  } else if constexpr (collider_traits::kHasIntersectWithRayFunc<T>) {
    if (absl::optional<RayHit> result = collider->Intersect(world_ray)) {
      out_intersections->push_back(*result);
    }
  } else if constexpr (collider_traits::kHasMultipleIntersectWithRayFunc<T>) {
    std::vector<RayHit> result = collider->Intersect(world_ray);
    out_intersections->insert(out_intersections->end(), result.begin(),
                              result.end());
  }
}

template <typename T>
void CollisionSystem<T>::IntersectPrecise(
    const DoubleRay& world_ray, Flags<CollisionMask> mask,
    std::vector<DoubleRayHit>* out_intersections) {
  GetComponentManager().template ForEach<T>(
      [&world_ray, &out_intersections, mask](T* collider) {
        IntersectColliderPrecise(collider, world_ray, mask, out_intersections);
      });
}

template <typename T>
void CollisionSystem<T>::IntersectColliderPrecise(
    T* collider, const DoubleRay& world_ray, Flags<CollisionMask> mask,
    std::vector<DoubleRayHit>* out_intersections) {
  if constexpr (collider_traits::kHasIsActiveFunc<T>) {
    if (!collider->IsActive()) {
      return;
    }
  }

  if constexpr (collider_traits::kHasTestCollisionFlagsFunc<T>) {
    if (!collider->TestCollisionFlags(mask)) {
      return;
    }
  }
  if constexpr (collider_traits::kHasIntersectWithRayFunc<T>) {
    if (auto result = collider->IntersectPrecise(world_ray)) {
      out_intersections->push_back(*result);
    }
  } else if constexpr (collider_traits::
                           kHasMultipleIntersectPreciseWithDoubleRayFunc<T>) {
    std::vector<DoubleRayHit> result = collider->IntersectPrecise(world_ray);
    out_intersections->insert(out_intersections->end(), result.begin(),
                              result.end());
  }
}

template <typename T>
void CollisionSystem<T>::IntersectPrecise(
    float2 screen_pos, Flags<CollisionMask> mask,
    std::vector<DoubleRayHit>* out_intersections) {
  const DoubleRay world_ray =
      details::GetWorldRayFromPixelPositionPrecise(GetView(), screen_pos);
  GetComponentManager().template ForEach<T>(
      [this, &screen_pos, &world_ray, &out_intersections, mask](T* collider) {
        IntersectColliderPrecise(collider, world_ray, screen_pos, mask,
                                 out_intersections);
      });
}

template <typename T>
void CollisionSystem<T>::IntersectColliderPrecise(
    T* collider, const DoubleRay& world_ray, float2 screen_pos,
    Flags<CollisionMask> mask, std::vector<DoubleRayHit>* out_intersections) {
  if constexpr (collider_traits::kHasIsActiveFunc<T>) {
    if (!collider->IsActive()) {
      return;
    }
  }

  if constexpr (collider_traits::kHasTestCollisionFlagsFunc<T>) {
    if (!collider->TestCollisionFlags(mask)) {
      return;
    }
  }
  if constexpr (collider_traits::kHasIntersectWithScreenPosFunc<T>) {
    if (auto result = collider->IntersectPrecise(screen_pos)) {
      out_intersections->push_back(*result);
    }
  } else if constexpr (collider_traits::
                           kHasMultipleIntersectPreciseWithScreenPosFunc<T>) {
    std::vector<DoubleRayHit> result = collider->IntersectPrecise(screen_pos);
    out_intersections->insert(out_intersections->end(), result.begin(),
                              result.end());
  } else if constexpr (collider_traits::kHasIntersectWithRayFunc<T>) {
    if (auto result = collider->IntersectPrecise(world_ray)) {
      out_intersections->push_back(*result);
    }
  } else if constexpr (collider_traits::
                           kHasMultipleIntersectPreciseWithDoubleRayFunc<T>) {
    std::vector<DoubleRayHit> result = collider->IntersectPrecise(world_ray);
    out_intersections->insert(out_intersections->end(), result.begin(),
                              result.end());
  }
}

template <typename T>
void CollisionSystem<T>::IntersectNode(NodeHandle node, const Ray& world_ray,
                                       Flags<CollisionMask> mask,
                                       std::vector<RayHit>* out_intersections) {
  ComponentHandle<T> collider = node->GetComponent<T>();
  if (!collider) {
    return;
  }

  IntersectCollider(collider.Get(), world_ray, mask, out_intersections);
}

template <typename T>
void CollisionSystem<T>::IntersectNode(NodeHandle node, float2 screen_pos,
                                       Flags<CollisionMask> mask,
                                       std::vector<RayHit>* out_intersections) {
  ComponentHandle<T> collider = node->GetComponent<T>();
  if (!collider) {
    return;
  }

  const Ray world_ray =
      details::GetWorldRayFromPixelPosition(GetView(), screen_pos);
  IntersectCollider(collider.Get(), world_ray, screen_pos, mask,
                    out_intersections);
}

template <typename T>
void CollisionSystem<T>::IntersectNodePrecise(
    NodeHandle node, const DoubleRay& world_ray, Flags<CollisionMask> mask,
    std::vector<DoubleRayHit>* out_intersections) {
  ComponentHandle<T> collider = node->GetComponent<T>();
  if (!collider) {
    return;
  }

  IntersectColliderPrecise(collider.Get(), world_ray, mask, out_intersections);
}

template <typename T>
void CollisionSystem<T>::IntersectNodePrecise(
    NodeHandle node, float2 screen_pos, Flags<CollisionMask> mask,
    std::vector<DoubleRayHit>* out_intersections) {
  ComponentHandle<T> collider = node->GetComponent<T>();
  if (!collider) {
    return;
  }

  const DoubleRay world_ray =
      details::GetWorldRayFromPixelPositionPrecise(GetView(), screen_pos);
  IntersectColliderPrecise(collider.Get(), world_ray, screen_pos, mask,
                           out_intersections);
}

#if IMP_RUNTIME(DEV)
template <typename T>
void CollisionSystem<T>::VisualizeCollidersForNode(
    NodeHandle node, VisualizationStyle visualization_style) {
  GetComponentManager().template ForEach<T>(
      [this, &node, &visualization_style](T* collider) {
        if constexpr (collider_traits::kHasIsActiveFunc<T>) {
          if (!collider->IsActive()) {
            return;
          }
        }

        if (collider->GetEntity() == node.GetEntity()) {
          if constexpr (collider_traits::kHasVisualizeFunc<T>) {
            collider->Visualize(visualization_style);
          }
        }
      });
}
#endif

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COLLISION_SYSTEM_H_
