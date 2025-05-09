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

#ifndef THIRD_PARTY_IMPRESS_CORE_GEOMETRY_CAPSULE_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_GEOMETRY_CAPSULE_HELPER_H_

#include <optional>

#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/geometry/closest_point.h"
#include "core/geometry/geometry_helper.h"
#include "core/geometry/shapes/capsule.h"
#include "core/geometry/shapes/sphere.h"
#include "core/math/vec.h"

namespace imp {
namespace geometry_internal {

// Returns the intersection between a ray and the inner surface of a sphere.
template <typename T>
std::optional<collision::RayIntersection<T>> SphereInnerSurfaceIntersectRay(
    const Sphere& sphere, const GenericRay<T>& ray) {
  TVec3<T> d = normalize(ray.direction);
  TVec3<T> m = ray.origin - sphere.center;
  T dm = dot(d, m);
  T mm = dot(m, m);
  T dist2 = mm - sphere.radius * sphere.radius;
  // Return if the ray starts outside the sphere and goes away.
  if (dist2 > 0 && dm > 0) {
    return std::nullopt;
  }

  T descriminant = dm * dm - dist2;
  // Return if the infinite line (that the ray is on) doesn't intersect the
  // sphere.
  if (descriminant < 0) {
    return std::nullopt;
  }

  T t = -dm + std::sqrt(descriminant);
  collision::RayIntersection<T> result;
  result.distance = t / length(ray.direction);
  result.collision_point = ray.origin + t * d;
  result.normal = normalize(sphere.center - result.collision_point);
  return result;
}
}  // namespace geometry_internal

// Test if a capsule intersects a ray.
template <typename T>
std::optional<collision::RayIntersection<T>> CapsuleIntersectsRay(
    const Capsule& capsule, const GenericRay<T>& ray);

template <typename T>
std::optional<collision::RayIntersection<T>> CapsuleIntersectsRay(
    const Capsule& capsule, const GenericRay<T>& ray) {
  // Adjust the ray origin, so that the capsule center is on the zx-plane at y =
  // 0.
  TVec3<T> origin = ray.origin - capsule.center;
  TVec3<T> direction = normalize(ray.direction);

  T r_squared = capsule.radius * capsule.radius;
  // -1 means the ray starts inside the capsule.
  T ray_start_in_capsule =
      DistanceFromLineSegment<T>(origin, -kUp * capsule.height / 2.0,
                                 kUp * capsule.height / 2.0) < capsule.radius
          ? -T(1)
          : T(1);

  // The center of the sphere that the ray may intersect.
  TVec3<T> closer_sphere_center = capsule.center;

  std::optional<T> t1 =
      ClosestPointOnRayToLine<T>(ray, GenericRay<T>(capsule.center, kUp));
  if (!t1.has_value()) {
    // The ray is parallel to the capsule. Find the hemisphere that the ray may
    // intersect.
    if (ray_start_in_capsule > 0) {
      origin.y > 0 ? closer_sphere_center.y += capsule.height / 2
                   : closer_sphere_center.y -= capsule.height / 2;
    } else {
      direction.y > 0 ? closer_sphere_center.y += capsule.height / 2
                      : closer_sphere_center.y -= capsule.height / 2;
    }
  } else {
    // p1 is the closest point on the ray to the capsule's center line.
    TVec3<T> p1 = (*t1) * direction + origin;
    T p1_squared = p1.x * p1.x + p1.z * p1.z;

    // The ray doesn't intersect the infinite cylinder.
    if (p1_squared > r_squared) {
      return std::nullopt;
    }

    // t2 is the distance from the ray origin to the intersection between the
    // ray and the infinite cylinder that the capsule is built from.
    T t2 = *t1 - sqrt((r_squared - p1_squared) /
                      (direction.x * direction.x + direction.z * direction.z)) *
                     ray_start_in_capsule;
    // l1 is the distance from the adjusted ray-cylinder intersection to the
    // zx-plane at y = 0.
    T l1 = origin.y + t2 * direction.y;

    collision::RayIntersection<T> result;
    // Return if the intersection is on the cylinder.
    if (abs(l1) <= capsule.height / 2) {
      result.distance = t2;
      result.collision_point = ray.origin + t2 * direction;
      result.normal =
          normalize(TVec3<T>(result.collision_point.x - capsule.center.x, 0,
                             result.collision_point.z - capsule.center.z)) *
          ray_start_in_capsule;
      return result;
    }
    // Find the hemisphere that the ray may intersect.
    l1 > 0 ? closer_sphere_center.y += capsule.height / 2
           : closer_sphere_center.y -= capsule.height / 2;
  }

  // Return the intersection between the ray and the hemisphere if it exists.
  if (ray_start_in_capsule < 0) {
    return geometry_internal::SphereInnerSurfaceIntersectRay<T>(
        Sphere(closer_sphere_center, capsule.radius), ray);
  }
  collision::RayIntersection<T> result;
  if (collision::SphereIntersectsRay<T>(
          Sphere(closer_sphere_center, capsule.radius), ray, &result.distance,
          &result.collision_point) == collision::Result::kDoesIntersect) {
    result.normal = normalize(result.collision_point - closer_sphere_center);
    return result;
  }

  return std::nullopt;
}
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GEOMETRY_CAPSULE_HELPER_H_
