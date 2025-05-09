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

#ifndef THIRD_PARTY_IMPRESS_CORE_COLLISION_RAY_H_
#define THIRD_PARTY_IMPRESS_CORE_COLLISION_RAY_H_

#include "core/math/vec.h"

namespace imp {

// Represents an infinite ray starting from an 'origin' and pointing in a
// 'direction'.
//
// GenericRay does NOT automatically normalize direction vectors. Please make
// sure that the direction vectors are pre-normalized to avoid unexpected
// results.
template <typename T>
struct GenericRay {
  GenericRay() : origin(0), direction(0, 0, -1) {}

  template <typename U>
  explicit GenericRay(const GenericRay<U>& other)
      : origin(other.origin), direction(other.direction) {}

  GenericRay(const TVec3<T>& origin, const TVec3<T>& direction)
      : origin(origin), direction(direction) {}

  // Calculates and returns a position along the ray at time t.
  // Please note that this function assumes direction vector is already
  // normalized and will return incorrect results if it's not normalized.
  TVec3<T> GetPointAt(T t) const { return origin + t * direction; }

  // Transforms the ray by the transtorm_mat and returns a copy.
  // Please note that if scale is applied, the resulting ray's direction
  // vector's length is very unlikely to be 1 and therefore needs to be
  // normalized.
  GenericRay GetTransformed(const TMat44<T>& transform_mat) const {
    return GenericRay((transform_mat * TVec4<T>{origin, 1}).xyz,
                      (transform_mat * TVec4<T>{direction, 0}).xyz);
  }

  TVec3<T> origin;
  TVec3<T> direction;

  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "GenericRay can only be construted as GenericRay<float> or "
                "GenericRay<double>.");
};

using Ray = GenericRay<float>;
using DoubleRay = GenericRay<double>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COLLISION_RAY_H_
