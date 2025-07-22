/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_SHAPE_MATCHERS_H_
#define THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_SHAPE_MATCHERS_H_

#include "gmock/gmock.h"
#include "core/geometry/shapes/rect.h"

namespace imp {

namespace imp_matchers {
MATCHER_P(EqualsRect, expected_rect,
          absl::StrFormat("%s equal to %v", negation ? "isn't" : "is",
                          expected_rect)) {
  *result_listener << " where the Rect is " << ToString(arg);
  return arg.center == expected_rect.center &&
         arg.half_extent == expected_rect.half_extent;
};

inline ::testing::Matcher<const Rect&> ToRectMatcher(const Rect& rect) {
  return EqualsRect(rect);
}

inline ::testing::Matcher<const Rect&> ToRectMatcher(
    const ::testing::Matcher<const Rect&>& rect_matcher) {
  return rect_matcher;
}

}  // namespace imp_matchers

// Returns a gMock matcher that matches the provided Rect.
template <typename RectMatcher>
inline ::testing::Matcher<const Rect&> RectIs(const RectMatcher matcher) {
  return imp_matchers::ToRectMatcher(matcher);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_SHAPE_MATCHERS_H_
