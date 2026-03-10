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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_MATH_VEC3_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_MATH_VEC3_H_

#include <algorithm>
#include <sstream>
#include <string>

#include "core/common/type_helpers.h"
#include "core/math/almost_equal_helper.h"
// IWYU pragma: begin_exports
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "filament/libs/math/include/math/mat4.h"
#include "filament/libs/math/include/math/vec2.h"
#include "filament/libs/math/include/math/vec3.h"
#include "filament/libs/math/include/math/vec4.h"
#include "filament/libs/mathio/include/mathio/ostream.h"
// IWYU pragma: end_exports

namespace imp {

using float2 = ::filament::math::float2;
using float3 = ::filament::math::float3;
using float4 = ::filament::math::float4;
using double2 = ::filament::math::double2;
using double3 = ::filament::math::double3;
using double4 = ::filament::math::double4;
using int2 = ::filament::math::int2;
using int3 = ::filament::math::int3;
using int4 = ::filament::math::int4;
using uint2 = ::filament::math::uint2;
using uint3 = ::filament::math::uint3;
using uint4 = ::filament::math::uint4;
using short2 = ::filament::math::short2;
using short3 = ::filament::math::short3;
using short4 = ::filament::math::short4;
using ushort2 = ::filament::math::ushort2;
using ushort3 = ::filament::math::ushort3;
using ushort4 = ::filament::math::ushort4;
using byte2 = ::filament::math::byte2;
using byte3 = ::filament::math::byte3;
using byte4 = ::filament::math::byte4;
using ubyte2 = ::filament::math::ubyte2;
using ubyte3 = ::filament::math::ubyte3;
using ubyte4 = ::filament::math::ubyte4;
using bool2 = ::filament::math::bool2;
using bool3 = ::filament::math::bool3;
using bool4 = ::filament::math::bool4;

template <typename T>
using TMat44 = ::filament::math::details::TMat44<T>;
template <typename T>
using TVec2 = ::filament::math::details::TVec2<T>;
template <typename T>
using TVec3 = ::filament::math::details::TVec3<T>;
template <typename T>
using TVec4 = ::filament::math::details::TVec4<T>;

inline constexpr float3 kForward = {0.0f, 0.0f, -1.0f};
inline constexpr float3 kBack = {0.0f, 0.0f, 1.0f};
inline constexpr float3 kUp = {0.0f, 1.0f, 0.0f};
inline constexpr float3 kDown = {0.0f, -1.0f, 0.0f};
inline constexpr float3 kRight = {1.0f, 0.0f, 0.0f};
inline constexpr float3 kLeft = {-1.0f, 0.0f, 0.0f};
inline constexpr float3 kOne3 = {1.0f, 1.0f, 1.0f};
inline constexpr float3 kZero3 = {0.0f, 0.0f, 0.0f};

inline constexpr float3 kXAxis3f = kRight;
inline constexpr float3 kYAxis3f = kUp;
inline constexpr float3 kZAxis3f = kBack;

inline constexpr float4 kOne4 = {1.0f, 1.0f, 1.0f, 1.0f};
inline constexpr float4 kZero4 = {0.0f, 0.0f, 0.0f, 0.0f};

inline constexpr float2 kOne2 = {1.0f, 1.0f};
inline constexpr float2 kZero2 = {0.0f, 0.0f};

template <typename T>
using EnableIfVector = std::enable_if_t<
    kIsAnyOf<T, float2, float3, float4, double2, double3, double4, int2, int3,
             int4, uint2, uint3, uint4, short2, short3, short4, ushort2,
             ushort3, ushort4, byte2, byte3, byte4, ubyte2, ubyte3, ubyte4,
             bool2, bool3, bool4>,
    int>;

template <typename T, EnableIfVector<T> = 0>
std::string ToString(const T& v) {
  std::ostringstream ss;
  ss << v;
  return ss.str();
}

// Gets the angle between two vectors in radians.
template <typename T, EnableIfVector<T> = 0>
float RadiansBetween(const T& lhs, const T& rhs) {
  float combined_length = length(lhs) * length(rhs);

  if (AlmostEqual<AlmostEqualKind::Default>(combined_length, 0.0f)) {
    return 0.0f;
  }

  float dot_product = dot(lhs, rhs);
  float cos = dot_product / combined_length;

  // Clamp due to floating point precision that could cause dot to be >
  // combinedLength. Which would cause acos to return NaN.
  cos = std::clamp(cos, -1.0f, 1.0f);
  return static_cast<float>(acos(cos));
}

}  // namespace imp

namespace filament::math::details {

template <typename Sink, typename T, ::imp::EnableIfVector<T> = 0>
void AbslStringify(Sink& sink, const T& vec) {
  sink.Append(::imp::ToString(vec));
}

}  // namespace filament::math::details

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_MATH_VEC3_H_
