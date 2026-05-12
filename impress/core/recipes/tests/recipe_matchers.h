// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_TESTS_RECIPE_MATCHERS_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_TESTS_RECIPE_MATCHERS_H_

#include <cassert>
#include <ostream>
#include <string>
#include <string_view>
#include <type_traits>

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "absl/types/variant.h"
#include "core/common/type_helpers.h"
#include "core/geometry/shapes/box.h"
#include "core/math/almost_equal.h"
#include "core/math/almost_equal_helper.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_writer.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/scene_handles/scene_handles.h"
#include "google/protobuf/util/message_differencer.h"

namespace imp {

namespace recipes_testing {

template <typename T>
constexpr bool kIsImpType = kIsAnyOf<T, NodeHandle, NodeSceneHandle>;

template <typename T>
constexpr bool kIsVecType =
    kIsAnyOf<T, float2, float3, float4, mat2f, mat3f, mat4f, quatf>;

template <typename T>
constexpr bool kIsNativeType =
    kIsAnyOf<T, int, float, bool, double, std::string_view, std::string>;

template <typename T>
constexpr bool kIsImpRecipeType =
    kIsAnyOf<T, Box, LiteralArray, LiteralTuple, LiteralMap, RecipeRayHit,
             ::google::protobuf::imp_proto::Any>;

template <typename T>
using EnableIfVecType = std::enable_if_t<kIsVecType<T>, int>;

template <typename T>
using EnableIfNativeType = std::enable_if_t<kIsNativeType<T>, int>;

template <typename T>
using EnableIfImpRecipeType = std::enable_if_t<kIsImpRecipeType<T>, int>;

// Visitor-type helper struct for testing if two variant types in the Recipe
// graph are equal.
template <typename Variant>
struct VariantEq {
  template <typename T, EnableIfVecType<T> = 0>
  bool operator()(const T& rhs) const {
    return AlmostEqual(std::get<T>(lhs), rhs);
  }

  template <typename T, EnableIfNativeType<T> = 0>
  bool operator()(const T& rhs) const {
    return std::get<T>(lhs) == rhs;
  }

  template <typename T, EnableIfImpRecipeType<T> = 0>
  bool operator()(const T& rhs) const {
    return recipe::ToString(std::get<T>(lhs)) == recipe::ToString(rhs);
  }

  bool operator()(const absl::monostate& rhs) const {
    return std::holds_alternative<absl::monostate>(lhs);
  }

  bool operator()(const google::protobuf::imp_proto::Any& rhs) const {
    ::google::protobuf::Any rhs_any;
    std::string data;
    proto::SerializeTo(&rhs, &data);
    rhs_any.ParseFromString(data);
    ::google::protobuf::Any lhs_any;
    proto::SerializeTo(&std::get<google::protobuf::imp_proto::Any>(lhs), &data);
    lhs_any.ParseFromString(data);
    return proto2::util::MessageDifferencer::Equals(lhs_any, rhs_any);
  }

  bool operator()(const NodeHandle& rhs) const {
    return std::get<NodeHandle>(lhs) == rhs;
  }

  bool operator()(const NodeSceneHandle& rhs) const {
    return std::get<NodeSceneHandle>(lhs) == rhs;
  }

  const Variant& lhs;
};

// Matcher for testing if two Recipe types are equal in both type and value, or
// in floating point types, if they are almost equal.
template <typename ExpectedT>
class RecipeTypeMatcher {
 public:
  using is_gtest_matcher = void;
  explicit RecipeTypeMatcher(const ExpectedT& expected) : expected_(expected) {}

  template <typename ActualT>
  bool MatchAndExplain(const ActualT& actual,
                       ::testing::MatchResultListener* listener) const {
    return MatchAndExplainImpl(actual, listener);
  }

  void DescribeTo(std::ostream* os) const {
    *os << "is equal to ";
    DescribeExpected(os);
  }

  void DescribeNegationTo(std::ostream* os) const {
    *os << "is not equal to ";
    DescribeExpected(os);
  }

 private:
  void DescribeExpected(std::ostream* os) const {
    if constexpr (kIsNativeType<ExpectedT>) {
      *os << expected_;
    } else {
      *os << recipe::ToString(expected_);
    }
  }

  template <typename ActualT>
  void ExplainTypeMismatch(::testing::MatchResultListener* listener) const {
    *listener << "Expected type " << typeid(ExpectedT).name()
              << " but got type " << typeid(ActualT).name();
  }

  template <typename ActualT>
  void ExplainValueMismatch(::testing::MatchResultListener* listener,
                            const ActualT& actual) const {
    if constexpr (kIsNativeType<ActualT> || kIsNativeType<ExpectedT>) {
      *listener << "Expected value " << expected_ << " but got value "
                << actual;
    } else {
      *listener << "Expected value " << recipe::ToString(expected_)
                << " but got value " << recipe::ToString(actual);
    }
  }

  template <typename ActualT>
  bool MatchAndExplainImpl(const ActualT& actual,
                           ::testing::MatchResultListener* listener) const {
    if constexpr (kIsVecType<ActualT> || kIsVecType<ExpectedT> ||
                  kIsAnyOf<ActualT, int, float, bool, double> ||
                  kIsAnyOf<ExpectedT, int, float, bool, double>) {
      if constexpr (!std::is_same_v<ActualT, ExpectedT>) {
        ExplainTypeMismatch<ActualT>(listener);
        return false;
      } else if (!AlmostEqual(actual, expected_)) {
        ExplainValueMismatch(listener, actual);
        return false;
      } else {
        return true;
      }
    } else if constexpr (kIsImpRecipeType<ActualT> ||
                         kIsImpRecipeType<ExpectedT>) {
      if constexpr (!std::is_same_v<ActualT, ExpectedT>) {
        ExplainTypeMismatch<ActualT>(listener);
        return false;
      } else if (recipe::ToString(actual) != recipe::ToString(expected_)) {
        ExplainValueMismatch(listener, actual);
        return false;
      } else {
        return true;
      }
    } else if constexpr (kIsImpType<ActualT> || kIsImpType<ExpectedT>) {
      if constexpr (!std::is_same_v<ActualT, ExpectedT>) {
        ExplainTypeMismatch<ActualT>(listener);
        return false;
      } else if (actual != expected_) {
        ExplainValueMismatch(listener, actual);
        return false;
      } else {
        return true;
      }
    } else {
      if (actual != expected_) {
        ExplainValueMismatch(listener, actual);
        return false;
      }
      return true;
    }
  }

  ExpectedT expected_;
};

template <typename T>
class LiteralMatcher {
 public:
  explicit LiteralMatcher(::testing::Matcher<const T&> matcher)
      : matcher_(std::move(matcher)) {}

  bool MatchAndExplain(const Literal& actual,
                       ::testing::MatchResultListener* listener) const {
    if (!std::holds_alternative<T>(actual.value)) {
      *listener << "whose value is not of the expected type";
      return false;
    }

    const bool match =
        matcher_.MatchAndExplain(std::get<T>(actual.value), listener);
    *listener << "whose value " << (match ? " matches" : " doesn't match");
    return match;
  }

  void DescribeTo(std::ostream* os) const {
    *os << "is a variant<> matching the value ";
    matcher_.DescribeTo(os);
  }

  void DescribeNegationTo(std::ostream* os) const {
    *os << "is a variant<> not matching the value ";
    matcher_.DescribeNegationTo(os);
  }

 private:
  const ::testing::Matcher<const T&> matcher_;
};

template <typename T>
inline ::testing::Matcher<const Literal&> ToLiteralMatcher(const T value) {
  return ::testing::MakePolymorphicMatcher(
      LiteralMatcher<T>(RecipeTypeMatcher<T>(value)));
}

template <typename T>
inline ::testing::Matcher<const Literal&> ToLiteralMatcher(
    const ::testing::Matcher<const T&>& matcher) {
  return ::testing::MakePolymorphicMatcher(LiteralMatcher<T>(matcher));
}

// Specialization for char* to convert it to string.
template <>
inline ::testing::Matcher<const Literal&> ToLiteralMatcher(const char* value) {
  return ::testing::MakePolymorphicMatcher(LiteralMatcher<std::string>(
      RecipeTypeMatcher<std::string>(std::string(value))));
}

// Checks if the matched value is equal to the expected value in both type and
// value, or in floating point types, if they are almost equal.
template <typename T>
inline RecipeTypeMatcher<T> RecipeEq(T expected) {
  return RecipeTypeMatcher<T>(expected);
}

}  // namespace recipes_testing

// Returns a gMock matcher that matches the provided Recipe Literal.
template <typename LiteralMatcher>
inline ::testing::Matcher<const Literal&> LiteralWith(
    const LiteralMatcher matcher) {
  return recipes_testing::ToLiteralMatcher(matcher);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_TESTS_RECIPE_MATCHERS_H_
