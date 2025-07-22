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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATH_MATH_H_
#define THIRD_PARTY_IMPRESS_CORE_MATH_MATH_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <type_traits>

// IWYU pragma: begin_exports
#include "absl/strings/str_format.h"
#include "filament/filament/include/filament/Box.h"
#include "core/math/almost_equal.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
// IWYU pragma: end_exports

namespace imp {

// Expose filament's Box (center, halfExtent) structure in Imp's namespace.
using Box = ::filament::Box;

std::string ToString(const Box& v);

template <
    typename T,
    std::enable_if_t<std::is_arithmetic_v<std::remove_reference_t<T>>, int> = 0>
inline constexpr float ToRadians(T degrees) {
  constexpr float kFloatPi = M_PI;
  return degrees * (kFloatPi / 180.0f);
}
template <
    typename T,
    std::enable_if_t<std::is_arithmetic_v<std::remove_reference_t<T>>, int> = 0>
inline constexpr float ToDegrees(T radians) {
  constexpr float kFloatPi = M_PI;
  return radians * (180.0f / kFloatPi);
}

inline constexpr double ToRadians(double degrees) {
  return degrees * (M_PI / 180.0);
}
inline constexpr double ToDegrees(double radians) {
  return radians * (180.0 / M_PI);
}

enum class Clamp { kNone, kNonNegative };

// A helper function that computes the floating-point remainder of the division
// operation value / divisor.
// This can also apply clamping to the result before returning.
// Internally this calls std::fmod.
template <typename T>
T FloatModulo(T value, T divisor, Clamp clamp_mode = Clamp::kNone) {
  if (clamp_mode == Clamp::kNonNegative) {
    return std::fmod(std::fmod(value, divisor) + divisor, divisor);
  }
  return std::fmod(value, divisor);
}

template <typename T>
TVec3<T> FloatModulo(TVec3<T> value, T divisor,
                     Clamp clamp_mode = Clamp::kNone) {
  value.x = FloatModulo(value.x, divisor, clamp_mode);
  value.y = FloatModulo(value.y, divisor, clamp_mode);
  value.z = FloatModulo(value.z, divisor, clamp_mode);
  return value;
}

// Fill the given filament matrix with a float* (must be size 16).
template <typename T>
void FillMat4(const float* a, filament::math::details::TMat44<T>* out_mat) {
  *out_mat = filament::math::details::TMat44<T>(a[0], a[1], a[2], a[3],    //
                                                a[4], a[5], a[6], a[7],    //
                                                a[8], a[9], a[10], a[11],  //
                                                a[12], a[13], a[14], a[15]);
}

template <typename T>
inline constexpr T saturate(T v) noexcept {
  return T(std::min(T(1), std::max(T(0), v)));
}

template <typename T>
inline constexpr T clamp(T v, T min, T max) noexcept {
  return T(std::min(max, std::max(min, v)));
}

// Handles mixing scalar and vector types.
template <typename T, typename K>
inline constexpr T mix(T x, T y, K a) noexcept {
  return T(x * (K(1) - a) + y * a);
}

// Handles lerping scalar and vector types.
template <typename T, typename K>
inline constexpr T lerp(T x, T y, K a) noexcept {
  return mix(x, y, a);
}

template <typename T>
inline constexpr T sign(T x) noexcept {
  return x < T(0) ? T(-1) : T(1);
}

// Returns only the fractional part of a float. Together with the truncate
// function, it can split a number between the whole part and the fractional
// part.
template <typename T>
inline constexpr T fraction(T v) noexcept {
  return v - std::floor(v);
}

// Applies the given unary function to each element of the given TVec
// Returns a TVec containing the transformed results.
template <template <typename T> class ImpType, typename T,
          typename Fn = T (*)(T), EnableIfVector<ImpType<T>> = 0>
constexpr ImpType<T> TransformVector(Fn fn, ImpType<T> vec) {
  for (int i = 0; i < ImpType<T>::SIZE; ++i) {
    vec[i] = fn(vec[i]);
  }
  return vec;
}

// Applies the given unary function to each element of the given TMatNN
// Returns a TMatNN containing the transformed results.
template <template <typename T> class ImpType, typename T,
          typename Fn = T (*)(T), EnableIfMatrix<ImpType<T>> = 0>
constexpr ImpType<T> TransformMatrix(Fn fn, ImpType<T> matrix) {
  for (int i = 0; i < ImpType<T>::NUM_COLS; ++i) {
    for (int j = 0; j < ImpType<T>::NUM_ROWS; ++j) {
      matrix[i][j] = fn(matrix[i][j]);
    }
  }
  return matrix;
}

// Linearly interpolates between trs1 and trs2 for the parameter x by performing
// lerp on translation and scale, and slerp on rotation.
template <typename T, typename R, typename S, typename U>
inline constexpr Transform<T, R, S> lerpTransform(
    const Transform<T, R, S>& trs1, Transform<T, R, S>& trs2, U x) noexcept {
  using TranslationType = filament::math::details::TVec3<T>;
  using RotationType = filament::math::details::TQuaternion<R>;
  using ScaleType = filament::math::details::TVec3<S>;

  auto t = TranslationType(imp::lerp(trs1.translation, trs2.translation, x));
  auto r = RotationType(
      slerp(normalize(trs1.rotation), normalize(trs2.rotation), x));
  auto s = ScaleType(imp::lerp(trs1.scale, trs2.scale, x));
  return Transform<T, R, S>(t, r, s);
}

// Returns the scale to apply to a node in order to achieve a
// distance-independent, constant size where a 1 meter object would fill the
// screen at screen_size_ratio 1 (100%). fovs is a float2 containing the
// horizontal and vertical fovs in radians.
inline float getScreenScale(float distance, float screen_size_ratio,
                            float2 fovs) {
  // Account for distance so further objects do not appear smaller.
  float scale = distance / (1.0 / screen_size_ratio);
  // Include the FOV to the scale to account for differences in aspect ratio.
  scale *= tan(std::min(fovs.x, fovs.y) / 2);
  return scale;
}

// Return new surface normal under transformation.
template <typename T>
TVec3<T> TransformedSurfaceNormal(TVec3<T> normal, const TMat44<T>& transform) {
  normal = normalize(normal);
  TVec3<T> origin = {0, 0, 0};
  TVec3<T> up = {0, 1, 0};
  TVec3<T> tangent;
  TVec3<T> binormal;

  if (AlmostEqual(normal, up)) {
    tangent = {1, 0, 0};
    binormal = {0, 0, 1};
  } else if (AlmostEqual(normal, -up)) {
    tangent = {0, 0, -1};
    binormal = {-1, 0, 0};
  } else {
    tangent = cross(normal, up);
    binormal = cross(tangent, normal);
  }

  origin = (transform * origin).xyz;
  tangent = (transform * tangent).xyz - origin;
  binormal = (transform * binormal).xyz - origin;

  return normalize(cross(binormal, tangent));
}

// The result can be used to align other objects' up direction from Y-up to the
// input normal. Do NOT use it to generate tangents for Filament mesh as it's
// packed in a different way from what Filament mesh tangents need.
template <typename T>
TQuaternion<T> TransformNormalToOrientation(TVec3<T> normal) {
  TMat44<T> transform;
  normal = normalize(normal);
  TVec3<T> tangent;
  TVec3<T> binormal;
  if (AlmostEqual(normal, kUp)) {
    tangent = {1, 0, 0};
    binormal = {0, 0, 1};
  } else if (AlmostEqual(normal, -kUp)) {
    tangent = {0, 0, -1};
    binormal = {-1, 0, 0};
  } else {
    tangent = normalize(cross(normal, kUp));
    binormal = normalize(cross(tangent, normal));
  }
  transform[0][0] = tangent.x;
  transform[0][1] = tangent.y;
  transform[0][2] = tangent.z;
  transform[1][0] = normal.x;
  transform[1][1] = normal.y;
  transform[1][2] = normal.z;
  transform[2][0] = binormal.x;
  transform[2][1] = binormal.y;
  transform[2][2] = binormal.z;
  return transform.toQuaternion();
}

// Convert a normal vector to tangents quaternion to be used in meshes.
// Use this function if you're converting normal to tagents for Filament mesh.
// The generated tangent & bitangent is always orthoganl to the normal, but
// there is no guarantee on their directions otherwise. So you probably should
// not do normal mapping with it, i.e. using a normal other than (0, 0, 1) in
// tangent space.
template <typename T>
TQuaternion<T> NormalToTangent(TVec3<T> normal) {
  const TVec3<T> temp_axis =
      std::abs(dot(normal, imp::kRight)) < 0.99f ? imp::kRight : imp::kUp;
  TVec3<T> fake_bitangent = cross(temp_axis, normal);

  // Gram-Schmidt orthonormalize.
  const TVec3<T> fake_tangent = temp_axis - normal * dot(normal, temp_axis);
  fake_bitangent = fake_bitangent -
                   fake_tangent * dot(fake_tangent, fake_bitangent) -
                   normal * dot(normal, fake_bitangent);

  filament::math::details::TMat33<T> m(fake_tangent, fake_bitangent, normal);
  return filament::math::details::TMat33<T>::packTangentFrame(m,
                                                              sizeof(int16_t));
}

mat3f MatrixFromUvTransform(float2 offset, float rotation, float2 scale);

// Retrieves offset, rotation and scale from a matrix.
void UvTransformFromMatrix(const mat3f& matrix, float2& offset, float& rotation,
                           float2& scale);

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_MATH_HELPERS_H_
