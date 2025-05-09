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

#include "core/geometry/geometry_helper.h"

#include <cmath>

#include "core/collision/ray.h"
#include "core/geometry/shapes/line_segment.h"
#include "core/math/vec.h"

namespace imp {
namespace {

constexpr float kEpsilon = 1e-6f;

}  // namespace

float DistanceFromPointToLine(const float3& point, const LineSegment& line) {
  return DistanceFromPointToLine(
      point, Ray(line.line_start, line.line_end - line.line_start));
}

float DistanceFromPointToLine(const float3& point, const Ray& line) {
  float3 vec = point - line.origin;
  float3 line_direction = normalize(line.direction);

  float ramp_length = norm(vec);
  if (ramp_length < kEpsilon) {
    return 0.0f;
  }
  float bottom_length = dot(vec, line_direction);

  return sqrt(ramp_length * ramp_length - bottom_length * bottom_length);
}

}  // namespace imp
