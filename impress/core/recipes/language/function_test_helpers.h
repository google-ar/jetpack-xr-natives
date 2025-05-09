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

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "absl/status/statusor.h"
#include "core/recipes/language/recipe_utils.h"

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_FUNCTION_TEST_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_FUNCTION_TEST_HELPERS_H_

namespace imp::recipes_testing {

using ::testing::Field;
using ::testing::UnorderedElementsAre;
using ::testing::VariantWith;
using ::testing::status::IsOkAndHolds;

template <typename SocketMatcher, typename ResultT>
auto FunctionResult(SocketMatcher socket_matcher, ResultT result) {
  return ::testing::Pair<SocketMatcher>(socket_matcher,
                                        VariantWith<ResultT>(result));
}

template <typename ResultT>
auto FunctionResult(ResultT result) {
  return FunctionResult(recipe::kDefaultOutputSocketName, result);
}

template <typename... Args>
::testing::Matcher<const absl::StatusOr<recipe::ReturnValue>&> IsOkAndValuesAre(
    Args... matchers) {
  return ::testing::status::IsOkAndHolds(
      Field(&recipe::ReturnValue::values, UnorderedElementsAre(matchers...)));
}

}  // namespace imp::recipes_testing

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_FUNCTION_TEST_HELPERS_H_
