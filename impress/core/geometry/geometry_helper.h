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

#ifndef THIRD_PARTY_IMPRESS_CORE_GEOMETRY_GEOMETRY_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_GEOMETRY_GEOMETRY_HELPER_H_

#include "core/collision/ray.h"
#include "core/geometry/shapes/box.h"
#include "core/geometry/shapes/line_segment.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"

namespace imp {

inline Rect ToRect(Box box) { return Rect{box.center.xy, box.halfExtent.xy}; }

inline Box ToBox(Rect rect) {
  return Box{{rect.center, 0.0f}, {rect.half_extent, 0.0f}};
}

// Calculates the distance from a point to an infinite line, while the line is
// represented by a line segment on it.
float DistanceFromPointToLine(const float3& point, const LineSegment& line);

// Calculates the distance from a point to an infinite line, while the line is
// represented by a ray lies on it.
float DistanceFromPointToLine(const float3& point, const Ray& line);

// Returns the signed distance from a point to a line segment.
template <typename T>
T DistanceFromLineSegment(const TVec3<T>& p, TVec3<T> a, TVec3<T> b) {
  TVec3<T> pa = p - a;
  TVec3<T> ba = b - a;
  T h = clamp(dot(pa, ba) / dot(ba, ba), T(0), T(1));
  return norm(pa - ba * h);
}
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GEOMETRY_GEOMETRY_HELPER_H_
