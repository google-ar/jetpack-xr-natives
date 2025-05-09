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

#include "core/geometry/closest_point.h"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>

#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/geometry/geometry_helper.h"
#include "core/geometry/shapes/box.h"
#include "core/geometry/shapes/line_segment.h"
#include "core/math/vec.h"

namespace imp {
namespace {

float SnapToBound(float free_point, float bound) {
  if (abs(free_point) <= abs(bound)) {
    return free_point;
  }
  return free_point / abs(free_point) * abs(bound);
}

float3 ClosestPointOnLineSegmentToRay(const LineSegment& line, const Ray& ray) {
  float3 direction = line.line_end - line.line_start;
  float tmax = norm(direction);
  if (abs(tmax) < std::numeric_limits<float>::epsilon()) {
    return line.line_start;
  }
  Ray ray_with_point = Ray(line.line_start, direction / tmax);

  std::optional<float> distance_from_line_start =
      ClosestPointOnRayToLine(ray_with_point, ray);

  if (!distance_from_line_start.has_value()) {
    return norm(ray.origin - line.line_start) < norm(ray.origin - line.line_end)
               ? line.line_start
               : line.line_end;
  }
  if (*distance_from_line_start < 0) {
    return line.line_start;
  }
  if (*distance_from_line_start > tmax) {
    return line.line_end;
  }

  return line.line_start + *distance_from_line_start * direction / tmax;
}
}  // namespace

float3 ClosestPointOnBoxToPoint(const Box& box, const float3& point) {
  float3 closest_point;
  float3 free_point = point - box.center;
  closest_point.x = SnapToBound(free_point.x, box.halfExtent.x);
  closest_point.y = SnapToBound(free_point.y, box.halfExtent.y);
  closest_point.z = SnapToBound(free_point.z, box.halfExtent.z);
  return closest_point + box.center;
}

float3 ClosestPointOnBoxToRay(const Box& box, const Ray& ray) {
  std::optional<collision::RayIntersection<float>> hit_check =
      collision::AABBIntersectsRay(box, ray);
  // Check if the ray intersects with the box.
  if (hit_check.has_value()) {
    return hit_check->collision_point;
  }

  float3 points[8] = {
      box.center - box.halfExtent,
      {box.center.x - box.halfExtent.x, box.center.y - box.halfExtent.y,
       box.center.z + box.halfExtent.z},
      {box.center.x - box.halfExtent.x, box.center.y + box.halfExtent.y,
       box.center.z + box.halfExtent.z},
      {box.center.x - box.halfExtent.x, box.center.y + box.halfExtent.y,
       box.center.z - box.halfExtent.z},
      {box.center.x + box.halfExtent.x, box.center.y - box.halfExtent.y,
       box.center.z - box.halfExtent.z},
      {box.center.x + box.halfExtent.x, box.center.y - box.halfExtent.y,
       box.center.z + box.halfExtent.z},
      box.center + box.halfExtent,
      {box.center.x + box.halfExtent.x, box.center.y + box.halfExtent.y,
       box.center.z - box.halfExtent.z}};

  LineSegment edges[12] = {
      LineSegment(points[0], points[1]), LineSegment(points[1], points[2]),
      LineSegment(points[2], points[3]), LineSegment(points[3], points[1]),
      LineSegment(points[4], points[5]), LineSegment(points[5], points[6]),
      LineSegment(points[6], points[7]), LineSegment(points[7], points[4]),
      LineSegment(points[0], points[4]), LineSegment(points[1], points[5]),
      LineSegment(points[2], points[6]), LineSegment(points[3], points[7]),
  };

  float distance = std::numeric_limits<float>::max();
  float3 closest_point;
  for (auto& edge : edges) {
    float3 point = ClosestPointOnLineSegmentToRay(edge, ray);
    if (DistanceFromPointToLine(point, ray) < distance) {
      distance = DistanceFromPointToLine(point, ray);
      closest_point = point;
    }
  }

  return closest_point;
}

}  // namespace imp
