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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATH_AABB_H_
#define THIRD_PARTY_IMPRESS_CORE_MATH_AABB_H_

#include "core/geometry/shapes/box.h"
#include "core/math/math.h"

namespace imp {
// Calculates aabb Box from input vertices.
class AabbCalculator {
 public:
  void AddVertex(float3 vertex) {
    min_ = min(vertex, min_);
    max_ = max(vertex, max_);
  }

  Box GetAabb() {
    float3 center = (max_ + min_) / 2.f;
    return {/*center=*/center, /*halfExtent=*/center - min_};
  }

 private:
  float3 min_ = std::numeric_limits<float>::max();
  float3 max_ = std::numeric_limits<float>::min();
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATH_AABB_H_
