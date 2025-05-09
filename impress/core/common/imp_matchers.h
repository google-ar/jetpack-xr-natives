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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_IMP_MATCHERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_IMP_MATCHERS_H_

#include <cstddef>
#include <ostream>
#include <type_traits>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "filament/libs/mathio/include/mathio/ostream.h"
#include "core/math/almost_equal_helper.h"
#include "core/math/math.h"  // IWYU pragma: keep

namespace imp {
namespace testing {

// gMock matcher that supports imp types.
template <AlmostEqualKind kind, typename ImpType>
class ImpFloatMatcher : public ::testing::MatcherInterface<ImpType> {
 public:
  explicit ImpFloatMatcher(const ImpType& expected) : expected_(expected) {}

  // Delegates to impl based on type.
  bool MatchAndExplain(
      ImpType actual, ::testing::MatchResultListener* listener) const override {
    return MatchAndExplainImpl(actual, listener);
  }

  // Prints a description of this matcher to the given ostream.
  void DescribeTo(std::ostream* os) const override {
    *os << "is almost equal to " << expected_;
  }

 private:
  // ImpType is already known when this class is created, so we need another
  // type to perform SFINAE.
  // Returns true iff the elements of the vector match each other.
  template <typename T = ImpType, EnableIfVector<T> = 0>
  bool MatchAndExplainImpl(const ImpType& actual,
                           ::testing::MatchResultListener* listener) const {
    for (int i = 0; i < ImpType::SIZE; ++i) {
      auto actual_value = actual[i];
      auto expected_value = expected_[i];
      if (!AlmostEqual<kind>(actual_value, expected_value)) {
        return false;
      }
    }
    return true;
  }

  // Returns true iff the quaterions match each other.
  template <typename T = ImpType, EnableIfQuaternion<T> = 0>
  bool MatchAndExplainImpl(const ImpType& actual,
                           ::testing::MatchResultListener* listener) const {
    return AlmostEqual<kind>(actual, expected_);
  }

  // Returns true iff the matrices match each other.
  template <typename T = ImpType, EnableIfMatrix<T> = 0>
  bool MatchAndExplainImpl(const ImpType& actual,
                           ::testing::MatchResultListener* listener) const {
    for (size_t c = 0; c < ImpType::NUM_COLS; ++c) {
      for (size_t r = 0; r < ImpType::NUM_ROWS; ++r) {
        auto actual_value = actual[c][r];
        auto expected_value = expected_[c][r];
        if (!AlmostEqual<kind>(actual_value, expected_value)) {
          return false;
        }
      }
    }
    return true;
  }

  // Returns true iff the scalars match each other.
  template <typename T = ImpType,
            std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
  bool MatchAndExplainImpl(const ImpType& actual,
                           ::testing::MatchResultListener* listener) const {
    return AlmostEqual<kind>(actual, expected_);
  }

  ImpType expected_;
};

// This returns a gMock matcher that tests for approximate equality between imp
// types within the error tolerance. This uses an element-wise comparison, and
// does not account for types with differing elements that have equivalent
// physical interpretations.
//
// const float3 v1(1.0f, 2.0f, 3.0f);
// const float3 v2(1.0f, 2.0f, 3.0f);
// const float3 v3(1.0f, 2.0f, 4.0f);
//
// EXPECT_THAT(v1, AlmostEqual(v2));
// EXPECT_THAT(v1, Not(AlmostEqual(v3)));
//
template <typename ImpType>
::testing::Matcher<ImpType> AlmostEqual(const ImpType& actual) {
  return ::testing::MakeMatcher(
      new ImpFloatMatcher<AlmostEqualKind::Default, ImpType>(actual));
}

template <typename ImpType>
::testing::Matcher<ImpType> AlmostEqualPrecise(const ImpType& actual) {
  return ::testing::MakeMatcher(
      new ImpFloatMatcher<AlmostEqualKind::Precise, ImpType>(actual));
}

// RoughlyEqual is like AlmostEqual, but with wider tolerances, because nobody
// likes listing 8 decimal places unless they have to.
template <typename ImpType>
::testing::Matcher<ImpType> RoughlyEqual(const ImpType& actual) {
  return ::testing::MakeMatcher(
      new ImpFloatMatcher<AlmostEqualKind::Rough, ImpType>(actual));
}

// Make the matchers compatible with Pointwise, UnorderedPointwise, etc.

MATCHER(AlmostEqual, "") {
  const auto& a = ::testing::get<0>(arg);
  const auto& b = ::testing::get<1>(arg);
  return ::testing::ExplainMatchResult(AlmostEqual(b), a, result_listener);
}

MATCHER(AlmostEqualPrecise, "") {
  const auto& a = ::testing::get<0>(arg);
  const auto& b = ::testing::get<1>(arg);
  return ::testing::ExplainMatchResult(AlmostEqualPrecise(b), a,
                                       result_listener);
}

MATCHER(RoughlyEqual, "") {
  const auto& a = ::testing::get<0>(arg);
  const auto& b = ::testing::get<1>(arg);
  return ::testing::ExplainMatchResult(RoughlyEqual(b), a, result_listener);
}

}  // namespace testing
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_IMP_MATCHERS_H_
