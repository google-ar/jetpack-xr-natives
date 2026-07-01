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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATH_RECT_H_
#define THIRD_PARTY_IMPRESS_CORE_MATH_RECT_H_

#include <string>

#include "absl/strings/str_format.h"
#include "core/math/vec.h"

namespace imp {
// Represents a 2D axis-aligned rectangle by its center and half-extent. For the
// 3D version, please see the Box class.
struct Rect {
  // Computes the lowest coordinates corner of the rectangle.
  constexpr float2 GetMin() const { return center - half_extent; }

  // Computes the largest coordinates corner of the rectangle.
  constexpr float2 GetMax() const { return center + half_extent; }

  // Returns a Rect that encompasses the two points.
  static constexpr Rect FromPoints(const float2& p1, const float2& p2) {
    const float2 min_pt = min(p1, p2);
    const float2 max_pt = max(p1, p2);
    return {
        .center = (min_pt + max_pt) * 0.5f,
        .half_extent = (max_pt - min_pt) * 0.5f,
    };
  }

  // Returns true if the given point is inside the rectangle.
  constexpr bool Contains(const float2& point) const {
    const float2 min_pt = GetMin();
    const float2 max_pt = GetMax();
    return point.x >= min_pt.x && point.x <= max_pt.x && point.y >= min_pt.y &&
           point.y <= max_pt.y;
  }

  // Returns true if the two rectangles intersect.
  constexpr bool Intersects(const Rect& other) const {
    const float2 min1 = GetMin();
    const float2 max1 = GetMax();
    const float2 min2 = other.GetMin();
    const float2 max2 = other.GetMax();
    return min1.x <= max2.x && max1.x >= min2.x && min1.y <= max2.y &&
           max1.y >= min2.y;
  }

  // Returns true if the given rectangle is fully contained within this one.
  constexpr bool Contains(const Rect& other) const {
    const float2 min1 = GetMin();
    const float2 max1 = GetMax();
    const float2 min2 = other.GetMin();
    const float2 max2 = other.GetMax();
    return min2.x >= min1.x && max2.x <= max1.x && min2.y >= min1.y &&
           max2.y <= max1.y;
  }

  // Returns the ratio of the rectangle's width to its height.
  constexpr float GetAspect() const { return half_extent.x / half_extent.y; }

  // Center of the 2D rectangle.
  float2 center;

  // Half extent from the center both axes.
  float2 half_extent;

  friend bool operator==(const Rect& a, const Rect& b) {
    return a.center == b.center && a.half_extent == b.half_extent;
  }

  template <typename Sink>
  friend void AbslStringify(Sink& sink, const Rect& rect) {
    absl::Format(&sink, "Rect (center: %s, halfExtent: %s)",
                 ToString(rect.center), ToString(rect.half_extent));
  }
};

std::string ToString(const Rect& rect);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATH_RECT_H_
