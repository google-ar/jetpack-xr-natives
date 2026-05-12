/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_CONSTANTS_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_CONSTANTS_H_

#include "absl/time/time.h"
#include "core/common/smooth.h"

namespace svxr {

constexpr auto kResetMinDuration = absl::Milliseconds(240);
// System-defined depth values.
inline constexpr auto kSystemMinimumDepth = 0.75f;
inline constexpr auto kSystemBaseline = 1.75f;
inline constexpr auto kSystemMaximumDepth = 5.0f;
// Scale offset to counter system scaling.
inline constexpr auto kScaleAboveMaxDepth = 0.8f;
inline constexpr auto kScaleAboveBaseline = 0.9f;
inline constexpr auto kScaleAtBaseline = 1.0f;
// Standard scale increment rate.
inline constexpr auto kScaleIncrementRate = 0.155f;

// Spec is 18 degrees, but 15 degrees feels better.
inline constexpr float kViewDropDegrees = 15.f;
inline constexpr float kMinSvNodeDistance = 1.5f;
inline constexpr float kViewRatio = 0.3f;

inline constexpr auto kSmallestModelSize = 0.01f;
inline constexpr auto kLargestModelSize = 70.f;
inline constexpr auto kModelSizeEpsilon = 0.001f;
inline constexpr auto kDistanceBasedScaleFactorMin = 1.0f;
inline constexpr auto kDistanceBasedScaleFactorMax = kLargestModelSize;
inline constexpr auto kFootprintSlop = 0.1f;
inline constexpr auto kPickupOffset = 0.025f;

constexpr auto kSoftAnchorPositionParameters =
    imp::SmoothParameters(.01f, 0, 0);
constexpr auto kSmoothSlowResolvingPositionParameters =
   imp::SmoothParameters(4.f, 40.f, 100.f);
constexpr auto kSmoothFastResolvingPositionParameters =
    imp::SmoothParameters(1000.f, 1000.f, 1000.f);

constexpr auto kSmoothResetScaleParameters =
    imp::SmoothParameters(25.f, 50.f, 50.f);
constexpr auto kSmoothManualScaleParameters =
    imp::SmoothParameters(400.f, 800.f, 800.f);
constexpr auto kSmoothRotationParameters =
    imp::SmoothParameters(50.f, 250.f, 25.f);

}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_CONSTANTS_H_
