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

#ifndef THIRD_PARTY_IMPRESS_CORE_COLLISION_PLANE_H_
#define THIRD_PARTY_IMPRESS_CORE_COLLISION_PLANE_H_

#include "core/math/vec.h"

namespace imp {

// Represents an infinite plane with a normal vector and the distance from
// (0, 0, 0) to a point on the plane along the normal vector.
//
// GenericPlane does not automatically normalize normal vectors. Please make
// sure that the direction vectors are pre-normalized to avoid unexpected
// results.
template <typename T>
struct GenericPlane {
  GenericPlane() : normal(0, 1, 0), distance(0) {}
  GenericPlane(const TVec3<T>& normal, T distance)
      : normal(normal), distance(distance) {}

  // Get the origin point on the plane.
  // Please note that this function will not work if the normal vector is not
  // normalzied.
  TVec3<T> GetOrigin() { return distance * normal; }

  TVec3<T> normal;
  T distance;

  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "GenericPlane can only be construted as GenericPlane<float> or "
                "GenericPlane<double>.");
};

using Plane = GenericPlane<float>;
using DoublePlane = GenericPlane<double>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COLLISION_PLANE_H_
