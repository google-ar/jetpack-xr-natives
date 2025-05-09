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

#include "core/ar/ar_plane.h"

#include "core/common/log.h"
#include "core/ar/ar_session_native.h"
#include "core/common/platform_helpers.h"

namespace imp {
namespace ar {

ArPlane::ArPlane(ArTrackableId id, TrackingState tracking_state,
                 mat4f transform, float2 extents, PlaneType plane_type,
                 std::vector<float3>&& vertices)
    : ArTrackable(id, kType, tracking_state, transform),
      extents_(extents),
      plane_type_(plane_type),
      vertices_(std::move(vertices)) {}

// Adapted from ArCore:
// (broken link)
// Adapted for supporting iOS and Android.
// Note the test point is in XZ coordinates. This causes some
// subtleties internally since treating XZ points as XY points results in
// left-handed geometry, however from the outside this is transparent.
bool ArPlane::IsPointInConvexPolygon2d(const std::vector<float3>& polygon_data,
                                       const float2& candidate_point) {
  int polygon_size = static_cast<int>(polygon_data.size());

  // Check that the polygon is not degenerate.
  if (polygon_size < 3) {
    IMP_LOG(imp::ERROR) << "Polygon must have at least three vertices; #vertices = "
               << polygon_size;
    return false;
  }

  // Load the last point, to link the end of the polygon with the beginning.
  float2 p0(polygon_data[polygon_size - 1].x, polygon_data[polygon_size - 1].z);

  for (int idx = 0; idx < polygon_size; ++idx) {
    // An adjacent point along the polygon.
    const float2 p1(polygon_data[idx].x, polygon_data[idx].z);

    // Check that the test point is to the left of each polygon side.
    const float2 v_side(p1.x - p0.x, p1.y - p0.y);
    const float2 v_point(candidate_point.x - p0.x, candidate_point.y - p0.y);

    // For convex ccw polygons, (v_side x v_point) is positive if the point
    // is in the polygon. The comparison is FLIPPED because these are actually
    // XZ points, not XY points, and X x Z = -Y.
    if ((v_side.x * v_point.y - v_point.x * v_side.y) > imp::kFltEpsilon) {
      return false;
    }
    // Move to the next point.
    p0 = p1;
  }
  return true;
}

bool ArPlane::IsPointInPlanePolygon(const imp::float3& world_point) const {
  auto inverse_transform = inverse(transform_);
  float3 local_point = (inverse_transform * float4(world_point, 1.0f)).xyz;
  float2 plane_point = float2(local_point.x, local_point.z);
  bool2 is_dimension_in_extents = lessThanEqual(abs(plane_point), extents_);
  if (!(is_dimension_in_extents.x && is_dimension_in_extents.y)) return false;
  return IsPointInConvexPolygon2d(vertices_, plane_point);
}
}  // namespace ar
}  // namespace imp
