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

#include "core/collision/collision_helpers.h"

#include <cmath>

#include "core/collision/plane.h"
#include "core/collision/ray.h"
#include "core/geometry/shapes/line_segment.h"
#include "core/geometry/shapes/rect.h"
#include "core/geometry/shapes/sphere.h"
#include "core/geometry/shapes/triangle.h"
#include "core/math/almost_equal_helper.h"
#include "core/math/math.h"
#include "core/math/vec.h"

namespace imp {
namespace collision {

Result PlaneIntersectsRay(const Plane& local_plane,
                          const float3& local_plane_min,
                          const float3& local_plane_max, const Ray& local_ray,
                          float3* out_col_point) {
  float3 collision_point;
  if (Result::kDoesNotIntersect ==
      PlaneIntersectsRay(local_plane, local_ray, &collision_point)) {
    return Result::kDoesNotIntersect;
  }

  // Tests if the point is inside the planes extents.
  imp::Box extents;
  extents.set(local_plane_min, local_plane_max);
  if (Result::kDoesNotIntersect ==
      AABBContainsPoint(extents, collision_point)) {
    return Result::kDoesNotIntersect;
  }

  *out_col_point = collision_point;
  return Result::kDoesIntersect;
}

Result AABBIntersectsLineSegment(const imp::Box& box,
                                 const LineSegment& line_segment) {
  const float3 box_center = box.center;
  const float3 half_extents = box.halfExtent;
  float3 segment_mid = 0.5f * (line_segment.line_start + line_segment.line_end);
  const float3 segment_dir = segment_mid - line_segment.line_end;
  // Translate segment into box space.
  segment_mid = segment_mid - box_center;

  // Early out's test against the AABB face normals for a separating axis.
  // Projection onto x-axis.
  float x_proj = std::fabs(segment_dir.x);
  if (std::fabs(segment_mid.x) > half_extents.x + x_proj) {
    return Result::kDoesNotIntersect;
  }
  // Projection onto y-axis.
  float y_proj = std::fabs(segment_dir.y);
  if (std::fabs(segment_mid.y) > half_extents.y + y_proj) {
    return Result::kDoesNotIntersect;
  }
  // Projection onto z-axis.
  float z_proj = std::fabs(segment_dir.z);
  if (std::fabs(segment_mid.z) > half_extents.z + z_proj) {
    return Result::kDoesNotIntersect;
  }

  // If the segment is parallel to an axis the folling checks can fail with a
  // false negative due to rounding error, giving them a pinch of epsilon to
  // compensate.
  x_proj += ::imp::kFltEpsilon;
  y_proj += ::imp::kFltEpsilon;
  z_proj += ::imp::kFltEpsilon;

  // Baked in cross-product of x-axis with segment projection.
  if (std::fabs(segment_mid.y * segment_dir.z - segment_mid.z * segment_dir.y) >
      half_extents.y * z_proj + half_extents.z * y_proj) {
    return Result::kDoesNotIntersect;
  }

  // Baked in cross-product of y-axis with segment projection.
  if (std::fabs(segment_mid.z * segment_dir.x - segment_mid.x * segment_dir.z) >
      half_extents.x * z_proj + half_extents.z * x_proj) {
    return Result::kDoesNotIntersect;
  }

  // Baked in cross-product of z-axis with segment projection.
  if (std::fabs(segment_mid.x * segment_dir.y - segment_mid.y * segment_dir.x) >
      half_extents.x * y_proj + half_extents.y * x_proj) {
    return Result::kDoesNotIntersect;
  }

  return Result::kDoesIntersect;
}

Result AABBContainsPoint(const imp::Box& box, const float3& test_point) {
  if ((std::abs(box.center.x - test_point.x) <= (box.halfExtent.x)) &&
      (std::abs(box.center.y - test_point.y) <= (box.halfExtent.y)) &&
      (std::abs(box.center.z - test_point.z) <= (box.halfExtent.z))) {
    return Result::kDoesIntersect;
  }
  return Result::kDoesNotIntersect;
}

Result AABBIntersectsAABB(const imp::Box& box_a, const imp::Box& box_b) {
  if ((std::abs(box_a.center.x - box_b.center.x) <=
       (box_a.halfExtent.x + box_b.halfExtent.x)) &&
      (std::abs(box_a.center.z - box_b.center.z) <=
       (box_a.halfExtent.z + box_b.halfExtent.z)) &&
      // Y is the least disciminatory so it's performed last.
      (std::abs(box_a.center.y - box_b.center.y) <=
       (box_a.halfExtent.y + box_b.halfExtent.y))) {
    return Result::kDoesIntersect;
  }
  return Result::kDoesNotIntersect;
}

float SquaredDistFromPointToAABB(const float3& point, const imp::Box& box) {
  float distSqrd = 0.0f;
  float3 min = box.center - box.halfExtent;
  float3 max = box.center + box.halfExtent;
  for (int i = 0; i < 3; ++i) {
    float value = point[i];
    if (value < min[i]) {
      distSqrd += (value - min[i]) * (value - min[i]);
    }
    if (value > max[i]) {
      distSqrd += (max[i] - value) * (max[i] - value);
    }
  }
  return distSqrd;
}

Result AABBIntersectsSphere(const imp::Box& box, const Sphere& sphere) {
  float distSqrd = SquaredDistFromPointToAABB(sphere.center, box);
  if (distSqrd > sphere.radius * sphere.radius) {
    return Result::kDoesNotIntersect;
  }
  return Result::kDoesIntersect;
}

Result AABBIntersectsTriangle(const imp::Box& box, const Triangle& tri) {
  // Note: This function is an implementation of the algorithm described in
  // Akenine-Möller, Tomas. "Fast 3D triangle-box overlap testing." Acm siggraph
  // 2005 courses. 2005. 8-es.
  // (broken link)

  // Tests AABB of the triangle against AABB of the box.
  filament::Aabb tri_aabb{
      .min = float3(fmin(tri.p0.x, fmin(tri.p1.x, tri.p2.x)),
                    fmin(tri.p0.y, fmin(tri.p1.y, tri.p2.y)),
                    fmin(tri.p0.z, fmin(tri.p1.z, tri.p2.z))),
      .max = float3(fmax(tri.p0.x, fmax(tri.p1.x, tri.p2.x)),
                    fmax(tri.p0.y, fmax(tri.p1.y, tri.p2.y)),
                    fmax(tri.p0.z, fmax(tri.p1.z, tri.p2.z)))};
  if (AABBIntersectsAABB(box, Box{.center = tri_aabb.center(),
                                  .halfExtent = tri_aabb.extent()}) ==
      Result::kDoesNotIntersect) {
    return Result::kDoesNotIntersect;
  }

  // Tests if the box intersects the triangle's plane.
  float3 normal = normalize(cross(tri.p1 - tri.p0, tri.p2 - tri.p0));

  float3 center_to_highest_corner =
      float3(abs(box.halfExtent.x) * (normal.x > 0.f ? 1.f : -1.f),
             abs(box.halfExtent.y) * (normal.y > 0.f ? 1.f : -1.f),
             abs(box.halfExtent.z) * (normal.z > 0.f ? 1.f : -1.f));
  if (dot(normal, box.center + center_to_highest_corner - tri.p0) *
          dot(normal, box.center - center_to_highest_corner - tri.p0) >
      0.f) {
    return Result::kDoesNotIntersect;
  }

  // Tests if triangle overlaps with the box on the xy-plane.
  float3 edge_0 = tri.p1 - tri.p0;
  float3 axy = float3(-edge_0.y, edge_0.x, 0.f);
  float box_proj_on_axy =
      box.halfExtent.x * abs(axy.x) + box.halfExtent.y * abs(axy.y);
  float p0_proj_on_axy = dot(tri.p0.xy - box.center.xy, axy.xy);
  float p2_proj_on_axy = dot(tri.p2.xy - box.center.xy, axy.xy);
  if (fmin(p0_proj_on_axy, p2_proj_on_axy) > box_proj_on_axy ||
      fmax(p0_proj_on_axy, p2_proj_on_axy) < -box_proj_on_axy) {
    return Result::kDoesNotIntersect;
  }

  // Tests if triangle overlaps with the box on the yz-plane.
  float3 ayz = float3(0.f, -edge_0.z, edge_0.y);
  float box_proj_on_ayz =
      box.halfExtent.y * abs(ayz.y) + box.halfExtent.z * abs(ayz.z);
  float p0_proj_on_ayz = dot(tri.p0.yz - box.center.yz, ayz.yz);
  float p2_proj_on_ayz = dot(tri.p2.yz - box.center.yz, ayz.yz);
  if (fmin(p0_proj_on_ayz, p2_proj_on_ayz) > box_proj_on_ayz ||
      fmax(p0_proj_on_ayz, p2_proj_on_ayz) < -box_proj_on_ayz) {
    return Result::kDoesNotIntersect;
  }

  // Tests if triangle overlaps with the box on the zx-plane.
  float3 azx = float3(edge_0.z, 0.f, -edge_0.x);
  float box_proj_on_azx =
      box.halfExtent.z * abs(azx.z) + box.halfExtent.x * abs(azx.x);
  float p0_proj_on_azx =
      dot(float2(tri.p0.z - box.center.z, tri.p0.x - box.center.x),
          float2(azx.z, azx.x));
  float p2_proj_on_azx =
      dot(float2(tri.p2.z - box.center.z, tri.p2.x - box.center.x),
          float2(azx.z, azx.x));
  if (fmin(p0_proj_on_azx, p2_proj_on_azx) > box_proj_on_azx ||
      fmax(p0_proj_on_azx, p2_proj_on_azx) < -box_proj_on_azx) {
    return Result::kDoesNotIntersect;
  }

  return Result::kDoesIntersect;
}

Result SphereContainsPoint(const Sphere& sphere, const float3& test_point) {
  float3 diff = sphere.center - test_point;
  if (dot(diff, diff) > sphere.radius * sphere.radius) {
    return Result::kDoesNotIntersect;
  }
  return Result::kDoesIntersect;
}

Result RectContainsPoint(const Rect& rect, const float2& test_point) {
  if ((std::abs(rect.center.x - test_point.x) <= (rect.half_extent.x)) &&
      (std::abs(rect.center.y - test_point.y) <= (rect.half_extent.y))) {
    return Result::kDoesIntersect;
  }
  return Result::kDoesNotIntersect;
}

Result Capsule2DContainPoint(const LineSegment& line_segment, float radius,
                             const float2& test_point) {
  float2 p = test_point;
  float2 a = line_segment.line_start.xy;
  float2 b = line_segment.line_end.xy;
  float2 ba = b - a;
  float2 pa = p - a;
  float ba_sq = dot(ba, ba);
  float distance;
  if (ba_sq == 0) {
    distance = norm(pa);
  } else {
    float proj = dot(pa, ba) / ba_sq;
    float h = fmin(fmax(proj, 0.), 1.);
    distance = norm(pa - h * ba);
  }

  if (distance <= radius) {
    return Result::kDoesIntersect;
  }
  return Result::kDoesNotIntersect;
}

}  // namespace collision
}  // namespace imp
