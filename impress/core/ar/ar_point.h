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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_POINT_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_POINT_H_

#include "core/ar/ar_trackable.h"

namespace imp {
namespace ar {

// The representation of a point in a cluster of points.
class ArPoint : public ArTrackable {
 public:
  constexpr static TrackableType kType = TrackableType::kPoint;

  // ARCore specific data that provides extra information on the tracking state
  // of a point for the purposes of instant-placement.
  enum class TrackingMethod : uint8_t {
    kUnavailable = 0,
    kDistanceGuess = (1 << 0),
    kRealDepth = (1 << 1),
    kDistanceGuess_kRealDepth = (kDistanceGuess | kRealDepth),
  };

  ArPoint()
      : ArTrackable(ArTrackableId::InvalidId(), kType, TrackingState::kStopped,
                    mat4f()) {}
  ArPoint(ArTrackableId id, TrackingState tracking_state,
          TrackingMethod tracking_method, const mat4f& transform,
          std::shared_ptr<void> optional_underlying = nullptr)
      : ArTrackable(id, kType, tracking_state, transform,
                    std::move(optional_underlying)),
        tracking_method_(tracking_method) {}

  // Returns the tracking method which provides extra information on the
  // tracking state of a point for the purposes of instant-placement.
  TrackingMethod GetTrackingMethod() const { return tracking_method_; }

 private:
  TrackingMethod tracking_method_ = TrackingMethod::kUnavailable;
};

// Helper to special case trackable types at compile time.
// template <typename T>
template <typename T>
constexpr bool IsPoint() {
  return std::is_same_v<T, ArPoint>;
}

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_POINT_H_
