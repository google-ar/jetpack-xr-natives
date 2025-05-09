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

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_NATURAL_NEIGHBOR_NATURAL_NEIGHBOR_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_NATURAL_NEIGHBOR_NATURAL_NEIGHBOR_H_

#include <optional>
#include <variant>
#include <vector>

#include "core/geometry/shapes/circle.h"
#include "core/math/vec.h"

// TODO: Move Natural Neighbor calculation to impress/extensions.

namespace imp {

struct LineSegment2d {
  LineSegment2d(const float2& start, const float2& end)
      : start(start), end(end) {}
  float2 start;
  float2 end;
};

struct Ray2d {
  Ray2d(const float2& origin, const float2& direction)
      : origin(origin), direction(direction) {}
  float2 origin;
  float2 direction;
};

using Edge2d = std::variant<LineSegment2d, Ray2d>;

// Represents partial result of Delaunay triangulation and Voronoi Diagram
// around the given cell center.
struct VoronoiCell {
  struct Neighbor {
    Neighbor(const float2& position, const Edge2d& edge)
        : position(position), voronoi_edge(edge) {}
    float2 position;
    // Corresponds to the neighbor points. The Voronoi edge is perpendicular to
    // the line define by cell_center and the corresponding neighbor point.
    Edge2d voronoi_edge;
  };

  float2 cell_center;
  std::vector<Neighbor> neighbors;
  bool is_cell_closed = true;

  // The loose bounding circle of the all points, can be used to truncate edges.
  Circle bounds;
};

// Finds the natural neighbors of the location among the points. Natural
// neighbors is referring to those points that are within the same triangle of
// of the location when the 2d point cloud is triangulated.
std::optional<VoronoiCell> CalculateNaturalNeighbors(
    float2 location_2d, std::vector<float2>& points_2d);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_NATURAL_NEIGHBOR_NATURAL_NEIGHBOR_H_
