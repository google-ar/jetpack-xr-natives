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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATH_TRANSFORM_H_
#define THIRD_PARTY_IMPRESS_CORE_MATH_TRANSFORM_H_

#include <limits>
#include <string>
#include <type_traits>

#include "core/common/log.h"
#include "absl/strings/str_format.h"
#include "filament/libs/math/include/math/mat3.h"
#include "filament/libs/math/include/math/mat4.h"
#include "filament/libs/math/include/math/quat.h"
#include "filament/libs/math/include/math/vec3.h"
#include "core/common/type_helpers.h"
#include "core/math/almost_equal_helper.h"

namespace imp {

// Scale-Rotation-Translation structure.  Represents an affine 4x4
// matrix in a more compact and blendable form.
// Translation, Rotation, and Scale may have different types, for example, in
// order to use higher precision doubles for the translation.
template <typename TranslationT, typename RotationT = TranslationT,
          typename ScaleT = RotationT>
struct Transform {
  template <typename U>
  using Vec3 = filament::math::details::TVec3<U>;
  template <typename U>
  using Quaternion = filament::math::details::TQuaternion<U>;
  template <typename U>
  using Mat4 = filament::math::details::TMat44<U>;
  template <typename U>
  using Mat3 = filament::math::details::TMat33<U>;

  enum class NoInit { kNoInit };
  using CommonT = std::common_type_t<TranslationT, RotationT, ScaleT>;
  using TranslationType = Vec3<TranslationT>;
  using RotationType = Quaternion<RotationT>;
  using ScaleType = Vec3<ScaleT>;

  // Leaves object in an uninitialized state. Use with caution.
  constexpr explicit Transform(NoInit) noexcept {}

  // Initialize to the equivalent of the identity matrix.
  constexpr Transform() noexcept;

  // Construct from another Transform.
  template <typename A, typename B, typename C>
  constexpr explicit Transform(const Transform<A, B, C>& rhs) noexcept;

  // Construct from a scale, rotation, and translation.
  template <typename A, typename B, typename C>
  constexpr Transform(const Vec3<A>& in_translation,
                      const Quaternion<B>& in_rotation,
                      const Vec3<C>& in_scale) noexcept;

  // Construct from a mat4.
  template <typename U>
  constexpr explicit Transform(const Mat4<U>& rhs) noexcept;

  // Coerce to a mat4.
  // TODO: Not currently usable because of a filament issue.
  constexpr explicit operator Mat4<CommonT>() const noexcept;
  // Convert to a mat4.
  template <typename U = CommonT>
  constexpr Mat4<U> AsMat4() const noexcept;

  TranslationType translation;
  RotationType rotation;
  ScaleType scale;
};

template <typename T, typename R, typename S>
constexpr Transform<T, R, S>::Transform() noexcept
    : translation(0), rotation(1, 0, 0, 0), scale(1) {}

template <typename T, typename R, typename S>
template <typename A, typename B, typename C>
constexpr Transform<T, R, S>::Transform(const Transform<A, B, C>& rhs) noexcept
    : translation(TranslationType(rhs.translation)),
      rotation(RotationType(rhs.rotation)),
      scale(ScaleType(rhs.scale)) {}

template <typename T, typename R, typename S>
template <typename A, typename B, typename C>
constexpr Transform<T, R, S>::Transform(const Vec3<A>& in_translation,
                                        const Quaternion<B>& in_rotation,
                                        const Vec3<C>& in_scale) noexcept
    : translation(TranslationType(in_translation)),
      rotation(RotationType(in_rotation)),
      scale(ScaleType(in_scale)) {}

template <typename T, typename R, typename S>
template <typename U>
constexpr Transform<T, R, S>::Transform(const Mat4<U>& rhs) noexcept {
  // Extract upper-left for determinant computation.
  const T a = T(rhs[0][0]);
  const T b = T(rhs[0][1]);
  const T c = T(rhs[0][2]);
  const T d = T(rhs[1][0]);
  const T e = T(rhs[1][1]);
  const T f = T(rhs[1][2]);
  const T g = T(rhs[2][0]);
  const T h = T(rhs[2][1]);
  const T i = T(rhs[2][2]);
  const T A = e * i - f * h;
  const T B = f * g - d * i;
  const T C = d * h - e * g;

  // Extract scale.
  const T det(a * A + b * B + c * C);
  T scalex = length(ScaleType({a, b, c}));
  T scaley = length(ScaleType({d, e, f}));
  T scalez = length(ScaleType({g, h, i}));
  auto s = ScaleType(scalex, scaley, scalez);
  if (det < 0) {
    s = -s;
  }
  scale = s;

  // Remove scale from the matrix if it is not close to zero.
  Mat3<U> upper_left = rhs.upperLeft();
  float eps = std::numeric_limits<float>::epsilon();
  if (s.x > eps && s.y > eps && s.z > eps) {
    upper_left[0] /= s.x;
    upper_left[1] /= s.y;
    upper_left[2] /= s.z;
  }

  // Extract rotation.
  rotation = RotationType(upper_left.toQuaternion());

  // Extract translation.
  translation = TranslationType(rhs[3].xyz);
}

template <typename T, typename R, typename S>
constexpr Transform<T, R, S>::operator Mat4<CommonT>() const noexcept {
  return AsMat4();
}

template <typename T, typename R, typename S>
template <typename U>
constexpr typename Transform<T, R, S>::template Mat4<U>
Transform<T, R, S>::AsMat4() const noexcept {
  const T tx = T(translation[0]);
  const T ty = T(translation[1]);
  const T tz = T(translation[2]);
  const R qx = R(rotation[0]);
  const R qy = R(rotation[1]);
  const R qz = R(rotation[2]);
  const R qw = R(rotation[3]);
  const S sx = S(scale[0]);
  const S sy = S(scale[1]);
  const S sz = S(scale[2]);
  return Mat4<U>(U((1 - 2 * qy * qy - 2 * qz * qz) * sx),  //
                 U((2 * qx * qy + 2 * qz * qw) * sx),      //
                 U((2 * qx * qz - 2 * qy * qw) * sx),      //
                 U(0.f),                                   //
                 U((2 * qx * qy - 2 * qz * qw) * sy),      //
                 U((1 - 2 * qx * qx - 2 * qz * qz) * sy),  //
                 U((2 * qy * qz + 2 * qx * qw) * sy),      //
                 U(0.f),                                   //
                 U((2 * qx * qz + 2 * qy * qw) * sz),      //
                 U((2 * qy * qz - 2 * qx * qw) * sz),      //
                 U((1 - 2 * qx * qx - 2 * qy * qy) * sz),  //
                 U(0.f),                                   //
                 U(tx), U(ty), U(tz), U(1.f));
}

// A transform with high precision for translation. Used often enough, and is
// long enough, that it's worth having a name for.
using PreciseTransform = Transform<double, float, float>;

template <typename T>
using EnableIfTransform = std::enable_if_t<
    kIsAnyOf<T, Transform<float>, Transform<double>, PreciseTransform>, int>;

template <typename T, EnableIfTransform<T> = 0>
std::string ToString(const T& transform) {
  return absl::StrFormat("< translation: %v, rotation: %v, scale: %v >",
                         transform.translation, transform.rotation,
                         transform.scale);
}

template <typename Sink, typename T, EnableIfTransform<T> = 0>
void AbslStringify(Sink& sink, const T& transform) {
  sink.Append(ToString(transform));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATH_TRANSFORM_H_
