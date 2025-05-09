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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_PLANE_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_PLANE_H_

#include <vector>

#include "core/ar/ar_trackable.h"
#include "core/math/math.h"

namespace imp {
namespace ar {
class ArSessionNative;
// The cross-platform representation of a flat surface.
class ArPlane : public ArTrackable {
  constexpr static TrackableType kType = TrackableType::kPlane;

 public:
  enum class PlaneType {
    kHorizontalUpFacing,
    kHorizontalDownFacing,
    kVertical,
  };

  ArPlane()
      : ArTrackable(ArTrackableId(0), kType, TrackingState::kStopped, mat4f()) {
  }
  ArPlane(ArTrackableId id, TrackingState tracking_state, mat4f transform,
          float2 extents, PlaneType plane_type, std::vector<float3>&& vertices);

  // Tests if a point is inside the polygon made from a planes vertices.
  bool IsPointInPlanePolygon(const float3& world_point) const;

  // The (x,z) size of the plane relative to the plane normal axis y.
  // Returns the x and z values as half-extents.
  float2 GetExtents() const { return extents_; }

  // The vertex geometry of the detected plane.
  const std::vector<float3>& GetVertices() const { return vertices_; }

  // Retuns kHorizontalUpFacing, kHorizontalDownFacing, or kVertical depending
  // on the planes normal vector.
  PlaneType GetType() const { return plane_type_; }

  static bool IsPointInConvexPolygon2d(const std::vector<float3>& polygon_data,
                                       const float2& candidate_point);

 private:
  float2 extents_;
  PlaneType plane_type_;
  std::vector<float3> vertices_;
};

// Helper to special case trackable types at compile time.
template <typename T>
constexpr bool IsPlane() {
  return std::is_same_v<T, ArPlane>;
}

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_PLANE_H_
