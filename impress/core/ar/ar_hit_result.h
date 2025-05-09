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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_HIT_RESULT_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_HIT_RESULT_H_

#include "core/ar/ar_trackable.h"
#include "core/math/math.h"

namespace imp {
namespace ar {

// The cross-platform representation of a hit result coming from an AR platform
// such as ARCore or ARKit.
class ArHitResult {
 public:
  ArHitResult(const quatf& rotation, const float3& world_position,
              float distance, ArTrackableId trackable_id)
      : rotation_(rotation),
        world_position_(world_position),
        distance_(distance),
        trackable_id_(trackable_id) {}
  // The normal of the surface that was hit in world space.
  float3 GetWorldNormal() const {
    constexpr float3 kUp = {0.0f, 1.0f, 0.0f};
    return rotation_ * kUp;
  }
  // The hit point in world space.
  float3 GetWorldHitPoint() const { return world_position_; }
  // The orientation of the hit point.
  quatf GetRotation() const { return rotation_; }
  // The distance from the ray origin to the hit point.
  float GetDistance() const { return distance_; }
  // The Id of the underlying platform specific trackable that was hit.
  ArTrackableId GetTrackableId() const { return trackable_id_; }

 private:
  quatf rotation_;
  float3 world_position_;
  float distance_;
  ArTrackableId trackable_id_;
};

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_IMPEL_AR_AR_HIT_RESULT_H_
