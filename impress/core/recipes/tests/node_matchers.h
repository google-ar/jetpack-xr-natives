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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_TESTS_NODE_MATCHERS_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_TESTS_NODE_MATCHERS_H_

#include <ostream>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_writer.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/scene_handles/scene_handles.h"
#include "google/protobuf/util/message_differencer.h"

// ((broken link)) These are matchers for Recipe stuff, and should belong in
// the Recipe library instead.
namespace imp::recipes_testing {

// Matcher for testing equality of RecipeGraph::NodeId
MATCHER_P(EqNodeId, node_id, "") { return arg.index == node_id.index; }

template <typename T>
using EnableIfImpType =
    std::enable_if_t<kIsAnyOf<T, float2, float3, float4, mat3f, mat4f, quatf>,
                     int>;

template <typename T>
using EnableIfNativeType = std::enable_if_t<
    kIsAnyOf<T, int, float, bool, double, std::string_view, std::string>, int>;

template <typename T>
using EnableIfImpRecipeType = std::enable_if_t<
    kIsAnyOf<T, Box, LiteralArray, LiteralTuple, LiteralMap, RecipeRayHit>,
    int>;

// Visitor-type helper struct for testing if two variant types in the Recipe
// graph are equal.
template <typename Variant>
struct VariantEq {
  template <typename T, EnableIfImpType<T> = 0>
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

class LiteralMatcher : public ::testing::MatcherInterface<const Literal&> {
 public:
  explicit LiteralMatcher(const Literal& expected) : expected_(expected) {}

  bool MatchAndExplain(const Literal& actual,
                       ::testing::MatchResultListener* listener) const override;

  void DescribeTo(std::ostream* os) const override;

 private:
  const Literal& expected_;
};

// Matcher class for testing equality of two ValueConnections
template <typename ValueType>
class ValueConnectionMatcher {
 public:
  explicit ValueConnectionMatcher(::testing::Matcher<const ValueType&> matcher)
      : matcher_(std::move(matcher)) {}

  template <typename ConnectionVariant>
  bool MatchAndExplain(const ConnectionVariant& actual,
                       ::testing::MatchResultListener* listener) const {
    if (!std::holds_alternative<ValueType>(actual.connection)) {
      *listener << "whose value is not of the expected type";
      return false;
    }

    const ValueType& elem = std::get<ValueType>(actual.connection);
    const bool match = matcher_.MatchAndExplain(elem, listener);
    *listener << "whose value " << (match ? " matches" : " doesn't match");
    return match;
  }

  void DescribeTo(std::ostream* os) const {
    *os << "is a variant<> and the value ";
    matcher_.DescribeTo(os);
  }

  void DescribeNegationTo(std::ostream* os) const {
    *os << "is a variant<> not matching the value ";
    matcher_.DescribeNegationTo(os);
  }

 private:
  const ::testing::Matcher<const ValueType&> matcher_;
};

// Matcher for RecipeGraph::Literal
::testing::PolymorphicMatcher<LiteralMatcher> EqualsLiteral(
    const Literal& literal);

// Matcher for RecipeGraph::ValueConnection with its variant class member
template <typename T>
::testing::PolymorphicMatcher<ValueConnectionMatcher<T>> ValueConnectionWith(
    const ::testing::Matcher<const T&>& matcher) {
  return ::testing::MakePolymorphicMatcher(ValueConnectionMatcher<T>(matcher));
}

}  // namespace imp::recipes_testing

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_TESTS_NODE_MATCHERS_H_
