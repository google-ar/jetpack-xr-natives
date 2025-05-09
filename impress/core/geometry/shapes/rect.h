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

#include "core/math/vec.h"

namespace imp {
// Represents a 2D axis-aligned rectangle by its center and half-extent. For the
// 3D version, please see the Box class.
struct Rect {
  // Computes the lowest coordinates corner of the rectangle.
  constexpr float2 GetMin() const { return center - half_extent; }

  // Computes the largest coordinates corner of the rectangle.
  constexpr float2 GetMax() const { return center + half_extent; }

  // Center of the 2D rectangle.
  float2 center;

  // Half extent from the center both axes.
  float2 half_extent;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATH_RECT_H_
