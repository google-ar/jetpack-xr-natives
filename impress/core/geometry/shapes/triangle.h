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

#ifndef THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_TRIANGLE_H_
#define THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_TRIANGLE_H_

#include "core/math/vec.h"

namespace imp {

// Represents a triangle where the 3 points correspond to its vertices
struct Triangle {
  Triangle() : p0(0.0f), p1(0.0f), p2(0.0f) {}
  Triangle(const float3& p0, const float3& p1, const float3& p2)
      : p0(p0), p1(p1), p2(p2) {}
  float3 p0;
  float3 p1;
  float3 p2;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_TRIANGLE_H_
