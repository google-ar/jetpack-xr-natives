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

#include "core/material_library/test_helpers.h"

#include <cstdint>
#include <ostream>

#include "filament/filament/include/filament/TextureSampler.h"

namespace imp {

TextureSamplerMatcher::TextureSamplerMatcher(
    const filament::TextureSampler& expected)
    : expected_(expected) {}

void TextureSamplerMatcher::DescribeTo(std::ostream* os) const {
  *os << "matches expected sampler";
}

bool TextureSamplerMatcher::MatchAndExplain(
    const filament::TextureSampler& arg, MatchResultListener* listener) const {
  bool result = true;
  if (arg.getMinFilter() != expected_.getMinFilter()) {
    *listener << "min_filter mismatch: "
              << static_cast<int32_t>(arg.getMinFilter())
              << " != " << static_cast<int32_t>(expected_.getMinFilter())
              << "\n";
    result = false;
  }
  if (arg.getMagFilter() != expected_.getMagFilter()) {
    *listener << "mag_filter mismatch: "
              << static_cast<int32_t>(arg.getMagFilter())
              << " != " << static_cast<int32_t>(expected_.getMagFilter())
              << "\n";
    result = false;
  }
  if (arg.getWrapModeS() != expected_.getWrapModeS()) {
    *listener << "wrap_mode_s mismatch: "
              << static_cast<int32_t>(arg.getWrapModeS())
              << " != " << static_cast<int32_t>(expected_.getWrapModeS())
              << "\n";
    result = false;
  }
  if (arg.getWrapModeT() != expected_.getWrapModeT()) {
    *listener << "wrap_mode_t mismatch: "
              << static_cast<int32_t>(arg.getWrapModeT())
              << " != " << static_cast<int32_t>(expected_.getWrapModeT())
              << "\n";
    result = false;
  }
  if (arg.getWrapModeR() != expected_.getWrapModeR()) {
    *listener << "wrap_mode_r mismatch: "
              << static_cast<int32_t>(arg.getWrapModeR())
              << " != " << static_cast<int32_t>(expected_.getWrapModeR())
              << "\n";
    result = false;
  }
  if (arg.getCompareMode() != expected_.getCompareMode()) {
    *listener << "compare_mode mismatch: "
              << static_cast<int32_t>(arg.getCompareMode())
              << " != " << static_cast<int32_t>(expected_.getCompareMode())
              << "\n";
    result = false;
  }
  if (arg.getCompareFunc() != expected_.getCompareFunc()) {
    *listener << "compare_func mismatch: "
              << static_cast<int32_t>(arg.getCompareFunc())
              << " != " << static_cast<int32_t>(expected_.getCompareFunc())
              << "\n";
    result = false;
  }
  if (arg.getAnisotropy() != expected_.getAnisotropy()) {
    *listener << "anisotropy mismatch: " << arg.getAnisotropy()
              << " != " << expected_.getAnisotropy() << "\n";
    result = false;
  }
  return result;
}

Matcher<const filament::TextureSampler&> SamplerEquals(
    const filament::TextureSampler& expected) {
  return Matcher<const filament::TextureSampler&>(
      new TextureSamplerMatcher(expected));
}

}  // namespace imp
