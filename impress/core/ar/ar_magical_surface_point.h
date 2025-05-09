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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_MAGICAL_SURFACE_POINT_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_MAGICAL_SURFACE_POINT_H_

#include "core/ar/ar_trackable.h"

namespace imp {
namespace ar {

// The representation of a point in a cluster of points.
class ArMagicalSurfacePoint : public ArTrackable {
 public:
  constexpr static TrackableType kType = TrackableType::kMagicalSurfacePoint;

  ArMagicalSurfacePoint()
      : ArTrackable(ArTrackableId::InvalidId(), kType, TrackingState::kStopped,
                    mat4f()) {}
  ArMagicalSurfacePoint(ArTrackableId id, TrackingState tracking_state,
                        const mat4f& transform,
                        std::shared_ptr<void> optional_underlying = nullptr)
      : ArTrackable(id, kType, tracking_state, transform,
                    std::move(optional_underlying)) {}
};

// Helper to special case trackable types at compile time.
// template <typename T>
template <typename T>
constexpr bool IsMagicalSurfacePoint() {
  return std::is_same_v<T, ArMagicalSurfacePoint>;
}

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_MAGICAL_SURFACE_POINT_H_
