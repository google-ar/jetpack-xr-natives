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

#ifndef THIRD_PARTY_IMPRESS_CORE_GEOMETRY_CLOSEST_POINT_H_
#define THIRD_PARTY_IMPRESS_CORE_GEOMETRY_CLOSEST_POINT_H_

#include <optional>

#include "core/collision/ray.h"
#include "core/geometry/shapes/box.h"
#include "core/math/vec.h"

namespace imp {

// Finds the closest point on the box to the point.
float3 ClosestPointOnBoxToPoint(const Box& box, const float3& point);

// Finds the closest point on the box to the ray.
float3 ClosestPointOnBoxToRay(const Box& box, const Ray& ray);

// Finds the closest point on the ray (first argument) to the reference line
// (infinite line, second argument). Returns a value "t" that the closest point
// is ray.origin + t * ray.direction. Returns std::nullopt if the reference line
// is parallel to the ray.
template <typename T>
std::optional<T> ClosestPointOnRayToLine(const GenericRay<T>& ray,
                                         const GenericRay<T>& reference_line);

template <typename T>
std::optional<T> ClosestPointOnRayToLine(const GenericRay<T>& ray,
                                         const GenericRay<T>& reference_line) {
  // The reference line.
  TVec3<T> a0 = reference_line.origin;
  TVec3<T> a = normalize(reference_line.direction);

  // The ray.
  TVec3<T> b0 = ray.origin;
  TVec3<T> b = normalize(ray.direction);

  // Calculate the "x" to minimize the distance of "b0+x*b" to Ray (a0,a) by:
  // Vector from b0+x*b to a0: v1 = (b0 + x * b - a0)
  // Rejection of v1 on (a0,a): v2 = v1 - dot(v1, a) * a
  // distance = ||v2||
  // find shortest distance by solving: d(distance^2)/dx = 0
  // therefore, "x = x_top / x_bottom" equals to the following:
  TVec3<T> c0 = b0 - a0;
  TVec3<T> A = b - dot(b, a) * a;
  TVec3<T> B = c0 - dot(c0, a) * a;
  T x_top = -dot(A, B);
  T x_bottom = dot(A, A);
  if (x_bottom == 0) {
    // This means the line parallels with the ray. Returns the anchor that is
    // closer to the ray's origin.
    return std::nullopt;
  }
  return x_top / x_bottom;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GEOMETRY_CLOSEST_POINT_H_
