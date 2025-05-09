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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_PRIOR_MAP_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_PRIOR_MAP_H_

#include <cstddef>

#include "core/config.h"

#if IMP_PLATFORM(ANDROID)

#include "third_party/arcore/ar/core/feature_macros.h"

#if ARCORE_FEATURE_ENABLED(placement_with_prior_map)

// This define allows the cross-platform code know if the ArPriorMap type can
// be used or not in compilation time.
#define IMP_PRIOR_MAP

#include <vector>

#include "core/ar/ar_trackable.h"
#include "core/math/math.h"

namespace imp {
namespace ar {
// The representation of an offline localization map.
// Instances of this class will only be available when the experimental arcore
// placement_with_prior_map feature is declared/used.
class ArPriorMap : public ArTrackable {
  constexpr static TrackableType kType = TrackableType::kPriorMap;

 public:
  ArPriorMap()
      : ArTrackable(ArTrackableId(0), kType, TrackingState::kStopped, mat4f()) {
  }
  ArPriorMap(ArTrackableId id, TrackingState tracking_state)
      : ArTrackable(id, kType, tracking_state, mat4f()) {}
  // This constructor initialize ArPriorMap with a transformation from session
  // to ADF space adf_t_session.
  ArPriorMap(ArTrackableId id, TrackingState tracking_state,
             Transform<float>& adf_t_session)
      : ArTrackable(id, kType, tracking_state, mat4f()) {
    adf_t_session_transformation_ = adf_t_session;
  }
  // Get transformation from session space to ADF space from
  // ArPriorMapTrackable;
  Transform<float> GetSessionToADFTransformation() {
    return adf_t_session_transformation_;
  }

 private:
  Transform<float> adf_t_session_transformation_;
};

// Helper to special case trackable types at compile time.
template <typename T>
constexpr bool IsPriorMap() {
  return std::is_same_v<T, ArPriorMap>;
}

}  // namespace ar
}  // namespace imp

#endif  // ARCORE_FEATURE_ENABLED(placement_with_prior_map)

#endif  // IMP_PLATFORM(ANDROID)

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_PRIOR_MAP_H_
