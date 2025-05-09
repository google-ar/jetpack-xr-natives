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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_TEST_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_TEST_HELPERS_H_

#include <ostream>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "filament/filament/include/filament/TextureSampler.h"

namespace imp {

using ::testing::Matcher;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;

class TextureSamplerMatcher
    : public MatcherInterface<const filament::TextureSampler&> {
 public:
  explicit TextureSamplerMatcher(const filament::TextureSampler& expected);

  void DescribeTo(std::ostream* os) const override;

  bool MatchAndExplain(const filament::TextureSampler& arg,
                       MatchResultListener* listener) const override;

 private:
  filament::TextureSampler expected_;
};

// Matcher for comparing two filament::TextureSampler objects.
Matcher<const filament::TextureSampler&> SamplerEquals(
    const filament::TextureSampler& expected);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_TEST_HELPERS_H_
