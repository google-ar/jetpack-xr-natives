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
#include <variant>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"

// ((broken link)) These are matchers for Recipe stuff, and should belong in
// the Recipe library instead.
namespace imp::recipes_testing {

// Matcher for testing equality of RecipeGraph::NodeId
MATCHER_P(EqNodeId, node_id, "") { return arg.index == node_id.index; }

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
::testing::Matcher<const Literal&> EqualsLiteral(const Literal& literal);

// Matcher for RecipeGraph::ValueConnection with its variant class member
template <typename T>
::testing::PolymorphicMatcher<ValueConnectionMatcher<T>> ValueConnectionWith(
    const ::testing::Matcher<const T&>& matcher) {
  return ::testing::MakePolymorphicMatcher(ValueConnectionMatcher<T>(matcher));
}

}  // namespace imp::recipes_testing

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_TESTS_NODE_MATCHERS_H_
