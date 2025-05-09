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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_TEST_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_TEST_HELPERS_H_

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "core/common/optional_error.h"

namespace imp {

::testing::AssertionResult IsNotError(const OptionalError& error);
::testing::AssertionResult AssertionFailure(const OptionalError& error);

// For usage with gmock matchers.
// For example:
//   EXPECT_THAT(fn(), IsNoError())
//   EXPECT_THAT(fn(), IsError("Message"))

MATCHER(IsNoError, "IsNoError()") { return arg.ok(); }

MATCHER_P(IsError, value, "IsError(" + ::testing::PrintToString(value) + ")") {
  if (arg.ok()) {
    return false;
  }

  return ::testing::Matches(value)(arg.message());
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_TEST_HELPERS_H_
