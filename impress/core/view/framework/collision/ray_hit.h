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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_COLLISION_RAY_HIT_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_COLLISION_RAY_HIT_H_

#include <optional>
#include <any>

#include "core/collision/ray.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"

namespace imp {
// Stores the results of a ray collision test.
template <typename T>
struct GenericRayHit {
  GenericRayHit() : distance(0.0f), world_orientation({}), world_point({}) {}
  GenericRayHit(T distance, const TQuaternion<T> &world_orientation,
                const TVec3<T> &world_point, NodeHandle node,
                std::optional<TVec3<T>> world_normal = std::nullopt,
                std::any meta_data = std::any())
      : distance(distance),
        world_orientation(world_orientation),
        world_point(world_point),
        node(node),
        world_normal(world_normal),
        meta_data(meta_data) {}
  template <typename T2>
  explicit GenericRayHit(const GenericRayHit<T2> &other) {
    distance = T(other.distance);
    world_orientation =
        TQuaternion<T>(other.world_orientation.x, other.world_orientation.y,
                       other.world_orientation.z, other.world_orientation.w);
    world_point =
        TVec3<T>(other.world_point.x, other.world_point.y, other.world_point.z);
    if (other.world_normal.has_value()) {
      world_normal = TVec3<T>(other.world_normal->x, other.world_normal->y,
                              other.world_normal->z);
    }
    meta_data = other.meta_data;
    node = other.node;
  }

  template <typename Data>
  const Data* GetMetaData() const {
    return std::any_cast<Data>(&meta_data);
  }

  // The distance from the ray origin to intersection point
  T distance;
  // The orientation of the collision.
  TQuaternion<T> world_orientation;
  // The point of collision in world space.
  TVec3<T> world_point;
  // Handle to the node collided with.
  NodeHandle node;
  // The normal of the surface on the collision point.
  std::optional<TVec3<T>> world_normal;
  // Additional meta data about this ray hit.
  std::any meta_data;
};

using RayHit = GenericRayHit<float>;
using DoubleRayHit = GenericRayHit<double>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_COLLISION_RAY_HIT_H_
