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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATH_ALMOST_EQUAL_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_MATH_ALMOST_EQUAL_HELPER_H_

#include <math.h>

#include <cstdio>
#include <type_traits>

namespace imp {

// AlmostEqual has an optional template argument controlling precision.
enum class AlmostEqualKind {
  Default,  // Very precise (you need roughly 7 digits of precision to match).
  Rough,    // Looser, for terse unit tests (roughly 3 digits of precision).
  Precise,  // More precise than default.
};

template <typename T>
using EnableIfFloatingPoint =
    std::enable_if_t<std::is_floating_point<T>::value, int>;

namespace internal {

// Due to errors of some operations, e.g. sin() or sqrt(), the accepted epsilons
// need to be larger than the minimum float epsilon of around 1.0E-07.
constexpr double FloatEpsilon(AlmostEqualKind kind) {
  switch (kind) {
    default:
    case AlmostEqualKind::Default:
      return 1.0E-06;
    case AlmostEqualKind::Rough:
      return 1.0E-03;
    case AlmostEqualKind::Precise:
      return 1.0E-09;
  }
}

// The Rough test uses a much larger MaxDelta; we want 0.0001 to count as 0.
constexpr double MaxDelta(AlmostEqualKind kind) {
  switch (kind) {
    default:
    case AlmostEqualKind::Default:
      return 1.0E-10;
    case AlmostEqualKind::Rough:
      return 5.0E-04;
    case AlmostEqualKind::Precise:
      return 1.0E-16;
  }
}

/**
 * Returns true if two floats are equal within a tolerance. Useful for comparing
 * floating point numbers while accounting for the limitations in floating point
 * precision.
 */
// (broken link)/
template <AlmostEqualKind kind, class T, EnableIfFloatingPoint<T> = 0>
bool AlmostEqualHelper(T lhs, T rhs) {
  // Check if the numbers are really close -- needed
  // when comparing numbers near zero.
  double diff = abs(lhs - rhs);
  if (diff <= MaxDelta(kind)) {
    return true;
  }

  lhs = abs(lhs);
  rhs = abs(rhs);
  double largest = fmax(lhs, rhs);

  if (diff <= largest * FloatEpsilon(kind)) {
    return true;
  }

  return false;
}
}  // namespace internal

constexpr float kFltEpsilon = internal::FloatEpsilon(AlmostEqualKind::Default);
constexpr double kPreciseFltEpsilon =
    internal::FloatEpsilon(AlmostEqualKind::Precise);

// Enabled if T is a floating point value.
template <AlmostEqualKind kind, typename T, EnableIfFloatingPoint<T> = 0>
bool AlmostEqual(const T& lhs, const T& rhs) {
  return internal::AlmostEqualHelper<kind>(lhs, rhs);
}

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_MATH_ALMOST_EQUAL_HELPER_H_
