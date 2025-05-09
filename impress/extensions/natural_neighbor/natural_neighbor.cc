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

#include "extensions/natural_neighbor/natural_neighbor.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <vector>

#include "core/common/robin_set.h"
#include "core/geometry/circumcircle.h"
#include "core/geometry/geometry_helper.h"
#include "core/geometry/shapes/circle.h"
#include "core/geometry/shapes/triangle.h"
#include "core/math/vec.h"

namespace imp {
namespace {

using Neighbor = VoronoiCell::Neighbor;

std::optional<int> FindNextTriangle(const Triangle& triangle,
                                    const std::vector<float3> points,
                                    bool clockwise) {
  float3 p0 = triangle.p0;
  float3 p1 = triangle.p1;
  float3 p2 = triangle.p2;
  if (!clockwise) {
    p1 = triangle.p2;
    p2 = triangle.p1;
  }
  float3 normal = cross(p1 - p0, p2 - p0);

  float radius = std::numeric_limits<float>::max();
  int next_point_index = -1;

  // In all the remaining points, find the point that makes a triangle that
  // 1. Contains the right(clockwise) / left(counter-clockwise) edge of the
  // input triangle.
  // 2. Does not overlap with the input triangle.
  // 3. has the smallest circumcircle.
  for (int i = 0; i < points.size(); ++i) {
    float3 point_direction = cross(p2 - p0, points[i] - p0);
    if (dot(point_direction, normal) <= 0) continue;
    std::optional<Circle> circle = GetCircumcircle({points[i], p0, p2});
    if (circle.has_value()) {
      if (circle->radius < radius) {
        radius = circle->radius;
        next_point_index = i;
      }
    }
    if (norm(points[i] - p0) > radius) {
      break;
    }
  }
  if (next_point_index == -1) {
    return std::nullopt;
  }
  return next_point_index;
}
}  // namespace

std::optional<VoronoiCell> CalculateNaturalNeighbors(
    float2 location_2d, std::vector<float2>& points_2d) {
  if (points_2d.size() < 2) {
    return std::nullopt;
  }

  // Extend points to 3d to assist calculation.
  float3 location = float3(location_2d.x, location_2d.y, 0.0f);
  std::vector<float3> points;
  points.reserve(points_2d.size());
  for (const float2& point : points_2d) {
    points.push_back(float3(point.x, point.y, 0.0f));
  }

  // Calculate a circular bound.
  float3 bound_center = kZero3;
  float bound_radius = 0.0f;
  for (const float3& point : points) {
    bound_center += point;
  }
  bound_center /= points.size();
  for (const float3& point : points) {
    float distance = norm(point - bound_center);
    if (distance > bound_radius) {
      bound_radius = distance;
    }
  }
  Circle bounds = Circle{.center = bound_center, .radius = bound_radius};

  // Sort the points from closest to farthest.
  std::sort(points.begin(), points.end(),
            [location](const float3& a, const float3& b) {
              return norm(a - location) < norm(b - location);
            });

  // Find the first triangle formed by
  // 1. the input location
  // 2. the closest point to it, which is the first neighbor
  // 3. another point such that the triangle formed by it creates the smallest
  // circumcircle
  float radius = std::numeric_limits<float>::max();
  int second_point_index = -1;
  for (int i = 1; i < points.size(); ++i) {
    std::optional<Circle> circle =
        GetCircumcircle({points[0], points[i], location});
    if (circle.has_value()) {
      if (circle->radius < radius) {
        radius = circle->radius;
        second_point_index = i;
      }
    }
    if (norm(points[i] - location) > radius) {
      break;
    }
  }
  if (second_point_index == -1) {
    return std::nullopt;
  }
  Triangle first_triangle(location, points[0], points[second_point_index]);

  // Calculate the first circle.
  std::optional<Circle> result = GetCircumcircle(first_triangle);
  Circle first_circumcircle = *result;

  RobinSet<int> visited;
  visited.insert({0, second_point_index});
  bool is_cell_closed = false;

  // Search for next triangle clockwise.
  // nns is a list of natural neighbors.
  std::vector<float3> nns{first_triangle.p2};
  // tcs is a list of circumcenter of the triangles around the location.
  std::vector<float3> tcs{first_circumcircle.center};
  Triangle triangle = first_triangle;
  for (int i = 0; i < points.size(); ++i) {
    std::optional<int> next_point_index =
        FindNextTriangle(triangle, points, true);
    if (!next_point_index.has_value()) {
      // Voronoi cell is open.
      break;
    }
    if (visited.contains(next_point_index.value())) {
      // Next neighbor is the first neighbor (a.k.a the closest point to the
      // location), the Voronoi cell is closed.
      is_cell_closed = true;
      triangle =
          Triangle(location, triangle.p2, points[next_point_index.value()]);
      auto circle = GetCircumcircle(triangle);
      tcs.push_back(circle->center);
      break;
    }
    visited.insert({next_point_index.value()});
    triangle =
        Triangle(location, triangle.p2, points[next_point_index.value()]);
    nns.push_back(points[next_point_index.value()]);
    auto circle = GetCircumcircle(triangle);
    tcs.push_back(circle->center);
  }

  // Records natural neighbors found clockwise around the location.
  VoronoiCell cell;
  cell.cell_center = location.xy;
  cell.bounds = bounds;
  for (int i = 0; i < nns.size() - 1; ++i) {
    cell.neighbors.push_back(
        Neighbor(nns[i].xy, LineSegment2d(tcs[i].xy, tcs[i + 1].xy)));
  }

  // When the location is within the convex hull of the points, its Voronoi cell
  // is closed. Closes the cell and return.
  if (is_cell_closed) {
    int last_neighbor_index = nns.size() - 1;
    cell.neighbors.push_back(
        Neighbor(nns[last_neighbor_index].xy,
                 LineSegment2d(tcs[last_neighbor_index + 1].xy,
                               tcs[last_neighbor_index].xy)));

    cell.neighbors.push_back(Neighbor(
        points[0].xy, LineSegment2d(tcs[0].xy, tcs[tcs.size() - 1].xy)));
    return cell;
  }

  // Deal with the clockwise open end.
  int last_neighbor_index = nns.size() - 1;
  float3 opposite = first_triangle.p1;
  if (nns.size() > 1) {
    opposite = nns[nns.size() - 2];
  }
  // This direction is perpendicular to line{last neighbor, location}.
  float3 direction = normalize(
      cross(cross(opposite - location, nns[nns.size() - 1] - location),
            nns[nns.size() - 1] - location));

  cell.neighbors.push_back(
      Neighbor(nns[last_neighbor_index].xy,
               Ray2d(tcs[tcs.size() - 1].xy, direction.xy)));
  cell.is_cell_closed = false;

  // Search for next triangle counter-clockwise.
  nns.clear();
  nns.push_back(first_triangle.p1);
  tcs.clear();
  tcs.push_back(first_circumcircle.center);

  triangle = first_triangle;
  for (int i = 0; i < points.size(); ++i) {
    std::optional<int> next_point_index =
        FindNextTriangle(triangle, points, false);
    if (!next_point_index.has_value()) {
      break;
    }
    visited.insert({next_point_index.value()});
    triangle =
        Triangle(location, points[next_point_index.value()], triangle.p1);
    nns.push_back(points[next_point_index.value()]);
    auto circle = GetCircumcircle(triangle);
    tcs.push_back(circle->center);
  }

  // Records natural neighbors that is found counter-clockwisely.
  for (int i = 0; i < nns.size() - 1; ++i) {
    cell.neighbors.push_back(
        Neighbor(nns[i].xy, LineSegment2d(tcs[i].xy, tcs[i + 1].xy)));
  }

  // Deal with the counter-clockwise open end.
  float3 opposite_2 = first_triangle.p2;
  if (nns.size() > 1) {
    opposite_2 = nns[nns.size() - 2];
  }
  float3 direction_2 = normalize(
      cross(cross(opposite_2 - location, nns[nns.size() - 1] - location),
            nns[nns.size() - 1] - location));

  cell.neighbors.push_back(Neighbor(
      nns[nns.size() - 1].xy, Ray2d(tcs[tcs.size() - 1].xy, direction_2.xy)));
  cell.is_cell_closed = false;

  return cell;
}

}  // namespace imp
