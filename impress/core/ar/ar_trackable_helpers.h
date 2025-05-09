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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_HELPERS_H_

#include <vector>

#include "core/ar/ar_anchor.h"
#include "core/ar/ar_face.h"
#include "core/ar/ar_magical_surface_point.h"
#include "core/ar/ar_plane.h"
#include "core/ar/ar_point.h"
#include "core/config.h"
#if IMP_PLATFORM(ANDROID)
#include "core/ar/ar_prior_map.h"
#endif
#include "core/ar/ar_trackable_id.h"
#include "core/common/robin_map.h"

namespace imp {
namespace ar {
enum class HitMode {
  kDistanceGuess,
  kRealDepth,
  kMagicalSurfacePoint,
};
// A tuple of supported trackable list types.
using TrackableTuple = std::tuple<std::vector<ArPoint>, std::vector<ArPlane>,
                                  std::vector<ArMagicalSurfacePoint>,
                                  std::vector<ArAnchor>, std::vector<ArFace>
#ifdef IMP_PRIOR_MAP
                                  ,
                                  std::vector<ArPriorMap>
#endif
                                  >;
// A tuple of supported trackable map types.
using TrackableMapTuple = std::tuple<
    RobinMap<ArTrackableId, ArPoint>, RobinMap<ArTrackableId, ArPlane>,
    RobinMap<ArTrackableId, ArMagicalSurfacePoint>,
    RobinMap<ArTrackableId, ArAnchor>, RobinMap<ArTrackableId, ArFace>
#ifdef IMP_PRIOR_MAP
    ,
    RobinMap<ArTrackableId, ArPriorMap>
#endif
    >;
}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_HELPERS_H_
