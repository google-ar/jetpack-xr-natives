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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COLLIDER_TRAITS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COLLIDER_TRAITS_H_

#include <type_traits>
#include <utility>
#include <vector>

#include "absl/types/optional.h"
#include "core/collision/collision_flags.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/math/vec.h"
#include "core/view/framework/collision/ray_hit.h"

namespace imp::collider_traits {
namespace internal {

// Uses SFINAE to detect if T has 'Visualize(VisualizationStyle)' function.
template <typename T,
          std::enable_if_t<
              std::is_same_v<void, decltype(std::declval<T>().Visualize(
                                       std::declval<VisualizationStyle>()))>,
              int> = 0>
static constexpr bool HasVisualizeFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasVisualizeFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has 'Intersect(Ray)' function.
template <typename T,
          std::enable_if_t<std::is_same_v<absl::optional<RayHit>,
                                          decltype(std::declval<T>().Intersect(
                                              std::declval<const Ray&>()))>,
                           int> = 0>
static constexpr bool HasIntersectWithRayFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasIntersectWithRayFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has 'Intersect(Ray)' function that returns
// multiple rayhits.
template <typename T,
          std::enable_if_t<std::is_same_v<std::vector<RayHit>,
                                          decltype(std::declval<T>().Intersect(
                                              std::declval<const Ray&>()))>,
                           int> = 0>
static constexpr bool HasMultipleIntersectWithRayFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasMultipleIntersectWithRayFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has 'IntersectPrecise(DoubleRay)' function.
template <
    typename T,
    std::enable_if_t<std::is_same_v<absl::optional<DoubleRayHit>,
                                    decltype(std::declval<T>().IntersectPrecise(
                                        std::declval<const DoubleRay&>()))>,
                     int> = 0>
static constexpr bool HasIntersectPreciseWithDoubleRayFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasIntersectPreciseWithDoubleRayFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has 'IntersectPrecise(DoubleRay)' function that
// returns multiple rayhits.
template <
    typename T,
    std::enable_if_t<std::is_same_v<std::vector<DoubleRayHit>,
                                    decltype(std::declval<T>().IntersectPrecise(
                                        std::declval<const DoubleRay&>()))>,
                     int> = 0>
static constexpr bool HasMultipleIntersectPreciseWithDoubleRayFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasMultipleIntersectPreciseWithDoubleRayFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has 'Intersect(float2)' function.
template <typename T,
          std::enable_if_t<std::is_same_v<absl::optional<RayHit>,
                                          decltype(std::declval<T>().Intersect(
                                              std::declval<float2>()))>,
                           int> = 0>
static constexpr bool HasIntersectWithScreenPosFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasIntersectWithScreenPosFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has 'Intersect(float2)' function that returns
// multiple rayhits.
template <typename T,
          std::enable_if_t<std::is_same_v<std::vector<RayHit>,
                                          decltype(std::declval<T>().Intersect(
                                              std::declval<float2>()))>,
                           int> = 0>
static constexpr bool HasMultipleIntersectWithScreenPosFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasMultipleIntersectWithScreenPosFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has 'IntersectPrecise(float2)' function.
template <
    typename T,
    std::enable_if_t<std::is_same_v<absl::optional<DoubleRayHit>,
                                    decltype(std::declval<T>().IntersectPrecise(
                                        std::declval<float2>()))>,
                     int> = 0>
static constexpr bool HasIntersectPreciseWithScreenPosFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasIntersectPreciseWithScreenPosFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has 'IntersectPrecise(float2)' function that
// returns multiple rayhits.
template <
    typename T,
    std::enable_if_t<std::is_same_v<std::vector<DoubleRayHit>,
                                    decltype(std::declval<T>().IntersectPrecise(
                                        std::declval<float2>()))>,
                     int> = 0>
static constexpr bool HasMultipleIntersectPreciseWithScreenPosFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasMultipleIntersectPreciseWithScreenPosFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has 'TestCollisionFlags' function.
template <
    typename T,
    std::enable_if_t<
        std::is_same_v<bool, decltype(std::declval<T>().TestCollisionFlags(
                                 std::declval<Flags<CollisionMask>>()))>,
        int> = 0>
static constexpr bool HasTestCollisionFlagsFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasTestCollisionFlagsFunc(...) {
  return false;
}

// Uses SFINAE to detect if T has 'IsActive()' function.
template <
    typename T,
    std::enable_if_t<
        std::is_same_v<bool, decltype(std::declval<T>().IsActive())>, int> = 0>
static constexpr bool HasIsActiveFunc(int) {
  return true;
}

template <typename T>
static constexpr bool HasIsActiveFunc(...) {
  return false;
}

}  // namespace internal

// Compile-time check to see if a collider has a
// 'void Visualize(VisualizationStyle)' function.
template <typename Collider>
static constexpr bool kHasVisualizeFunc =
    internal::HasVisualizeFunc<Collider>(0);

// Compile-time check to see if a collider has an
// 'absl::optional<RayHit> Intersect(Ray)' function.
template <typename Collider>
static constexpr bool kHasIntersectWithRayFunc =
    internal::HasIntersectWithRayFunc<Collider>(0);

// Compile-time check to see if a collider has an
// 'std::vector<RayHit> Intersect(Ray)' function.
template <typename Collider>
static constexpr bool kHasMultipleIntersectWithRayFunc =
    internal::HasMultipleIntersectWithRayFunc<Collider>(0);

// Compile-time check to see if a collider has an
// 'absl::optional<RayHit> Intersect(float2)' function.
template <typename Collider>
static constexpr bool kHasIntersectWithScreenPosFunc =
    internal::HasIntersectWithScreenPosFunc<Collider>(0);

// Compile-time check to see if a collider has an
// 'std::vector<RayHit> Intersect(float2)' function.
template <typename Collider>
static constexpr bool kHasMultipleIntersectWithScreenPosFunc =
    internal::HasMultipleIntersectWithScreenPosFunc<Collider>(0);

// Compile-time check ot see if a collider has an
// 'absl::optional<DoubleRayHit> IntersectPrecise(DoubleRay)' function.
template <typename Collider>
static constexpr bool kHasIntersectPreciseWithDoubleRayFunc =
    internal::HasIntersectPreciseWithDoubleRayFunc<Collider>(0);

// Compile-time check ot see if a collider has an
// 'std::vector<DoubleRayHit> IntersectPrecise(DoubleRay)' function.
template <typename Collider>
static constexpr bool kHasMultipleIntersectPreciseWithDoubleRayFunc =
    internal::HasMultipleIntersectPreciseWithDoubleRayFunc<Collider>(0);

// Compile-time check to see if a collider has an
// 'absl::optional<DoubleRayHit> IntersectPrecise(float2)' function.
template <typename Collider>
static constexpr bool kHasIntersectPreciseWithScreenPosFunc =
    internal::HasIntersectPreciseWithScreenPosFunc<Collider>(0);

// Compile-time check to see if a collider has an
// 'std::vector<DoubleRayHit> IntersectPrecise(float2)' function.
template <typename Collider>
static constexpr bool kHasMultipleIntersectPreciseWithScreenPosFunc =
    internal::HasMultipleIntersectPreciseWithScreenPosFunc<Collider>(0);

// Compile-time check to see if a collider has a 'TestCollisionFlags' function.
template <typename Collider>
static constexpr bool kHasTestCollisionFlagsFunc =
    internal::HasTestCollisionFlagsFunc<Collider>(0);

// Compile-time check to see if a collider has a 'IsActive' function.
template <typename Collider>
static constexpr bool kHasIsActiveFunc = internal::HasIsActiveFunc<Collider>(0);

}  // namespace imp::collider_traits

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_COLLISION_COLLIDER_TRAITS_H_
