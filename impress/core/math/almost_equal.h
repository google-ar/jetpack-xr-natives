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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATH_ALMOST_EQUAL_H_
#define THIRD_PARTY_IMPRESS_CORE_MATH_ALMOST_EQUAL_H_

#include <math.h>

#include <cstdlib>

#include "core/math/almost_equal_helper.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"

namespace imp {

// Enabled if T is a vector type.
template <AlmostEqualKind kKind, typename T, EnableIfVector<T> = 0>
bool AlmostEqual(T const& lhs, T const& rhs) {
  for (int i = 0; i < T::SIZE; ++i) {
    if (!internal::AlmostEqualHelper<kKind>(lhs[i], rhs[i])) {
      return false;
    }
  }
  return true;
}

// Enabled if T is a matrix.
template <AlmostEqualKind kKind, typename T, EnableIfMatrix<T> = 0>
bool AlmostEqual(const T& lhs, const T& rhs) {
  for (int i = 0; i < T::NUM_ROWS; ++i) {
    for (int j = 0; j < T::NUM_COLS; ++j) {
      if (!internal::AlmostEqualHelper<kKind>(lhs[i][j], rhs[i][j])) {
        return false;
      }
    }
  }
  return true;
}

// Enabled if T is a quaternion.
template <AlmostEqualKind kKind, typename T, EnableIfQuaternion<T> = 0>
bool AlmostEqual(const T& lhs, const T& rhs) {
  typename T::value_type lhs_length = norm(lhs);
  typename T::value_type rhs_length = norm(rhs);
  typename T::value_type dot_value = abs(dot(lhs, rhs));

  // If the lengths are not equal, the quaternions are not equal, since they
  // imply different scales.
  if (!internal::AlmostEqualHelper<kKind>(lhs_length, rhs_length)) {
    return false;
  }

  // The normalized quaternions are equal if the dot product is 1.0, which means
  // they imply the same rotation. Multiplied the lengths on the right for
  // better performance.
  return internal::AlmostEqualHelper<kKind>(
      dot_value, typename T::value_type(lhs_length * rhs_length));
}

// Enabled if T is a transform.
template <AlmostEqualKind kKind, typename T, EnableIfTransform<T> = 0>
bool AlmostEqual(const T& lhs, const T& rhs) {
  return AlmostEqual<kKind>(lhs.translation, rhs.translation) &&
         AlmostEqual<kKind>(lhs.rotation, rhs.rotation) &&
         AlmostEqual<kKind>(lhs.scale, rhs.scale);
}

// Enabled if T is a float type.
template <AlmostEqualKind kKind>
bool AlmostEqual(float lhs, float rhs) {
  return internal::AlmostEqualHelper<kKind>(lhs, rhs);
}

// Delegate the default case to specializations
template <typename T>
bool AlmostEqual(const T& lhs, const T& rhs) {
  return AlmostEqual<AlmostEqualKind::Default>(lhs, rhs);
}

// Delegate the rough case to specializations
template <typename T>
bool RoughlyEqual(const T& lhs, const T& rhs) {
  return AlmostEqual<AlmostEqualKind::Rough>(lhs, rhs);
}

// Delegate the precise case to specializations
template <typename T>
bool AlmostEqualPrecise(const T& lhs, const T& rhs) {
  return AlmostEqual<AlmostEqualKind::Precise>(lhs, rhs);
}

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_MATH_ALMOST_EQUAL_H_
