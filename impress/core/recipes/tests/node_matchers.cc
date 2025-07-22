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

#include "core/recipes/tests/node_matchers.h"

#include <ostream>
#include <string>
#include <variant>

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "absl/types/variant.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_writer.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/recipes/tests/recipe_matchers.h"
#include "core/scene_handles/scene_handles.h"

namespace imp::recipes_testing {

namespace {

struct ValueVariant {
  std::ostream& out;

  template <typename T, EnableIfImpType<T> = 0>
  std::ostream& operator()(const T& val) const {
    out << ToString(val);
    return out;
  }

  template <typename T, EnableIfNativeType<T> = 0>
  std::ostream& operator()(const T& val) const {
    out << val;
    return out;
  }

  template <typename T, EnableIfImpRecipeType<T> = 0>
  std::ostream& operator()(const T& val) const {
    out << recipe::ToString(val);
    return out;
  }

  std::ostream& operator()(const absl::monostate& val) const { return out; }

  std::ostream& operator()(const google::protobuf::imp_proto::Any& val) const {
    std::string data;
    proto::SerializeTo(&val, &data);
    out << data;
    return out;
  }

  std::ostream& operator()(NodeHandle val) const {
    return out << "NodeHandle entity " << val.GetEntity().getId();
  }

  std::ostream& operator()(NodeSceneHandle val) const {
    return out << "NodeSceneHandle entity " << val->GetEntity().getId();
  }
};

}  // namespace

bool LiteralMatcher::MatchAndExplain(
    const Literal& actual, ::testing::MatchResultListener* listener) const {
  return expected_.value.index() == actual.value.index() &&
         std::visit(VariantEq{expected_.value}, actual.value);
}

void LiteralMatcher::DescribeTo(std::ostream* os) const {
  *os << " with value";
  std::visit(ValueVariant{*os}, expected_.value);
}

::testing::Matcher<const Literal&> EqualsLiteral(const Literal& literal) {
  return ::testing::MakeMatcher(new LiteralMatcher(literal));
}

}  // namespace imp::recipes_testing
