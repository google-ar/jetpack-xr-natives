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

#include "core/geometry/circumcircle.h"

#include <optional>

#include "core/collision/ray.h"
#include "core/geometry/closest_point.h"
#include "core/geometry/shapes/circle.h"
#include "core/geometry/shapes/triangle.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"

namespace imp {

std::optional<Circle> GetCircumcircle(const Triangle& triangle) {
  float3 a = triangle.p0;
  float3 b = triangle.p1;
  float3 c = triangle.p2;
  float3 ab = b - a;
  float3 ac = c - a;
  float3 normal = cross(ab, ac);

  // At least two of the points are overlapping, or the three points are
  // collinear.
  if (AlmostEqual(a, b) || AlmostEqual(a, c) || AlmostEqual(b, c) ||
      AlmostEqual(normal, kZero3)) {
    return std::nullopt;
  }

  float3 midpoint_ab = (a + b) / 2.0f;
  float3 bisector_line_1_direction = normalize(cross(ab, normal));
  Ray bisector_line_1(midpoint_ab, bisector_line_1_direction);
  Ray bisector_line_2((a + c) / 2.0f, normalize(cross(ac, normal)));

  std::optional<float3> t =
      ClosestPointOnRayToLine(bisector_line_1, bisector_line_2);
  float3 circumcenter = midpoint_ab + *t * bisector_line_1_direction;
  return Circle{.center = circumcenter,
                .normal = normal,
                .radius = norm(circumcenter - a)};
}

}  // namespace imp
