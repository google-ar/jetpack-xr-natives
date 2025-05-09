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

#ifndef THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_SPHERE_H_
#define THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_SPHERE_H_


#include "absl/strings/str_format.h"
#include "core/math/vec.h"

namespace imp {

// Represents a sphere by its center
struct Sphere {
  Sphere() : center(0.0f, 0.0f, 0.0f), radius(1.0f) {}
  Sphere(const float3& center, float radius) : center(center), radius(radius) {}
  float3 center;
  float radius;

  template <typename Sink>
  friend void AbslStringify(Sink& sink, const Sphere& sphere);
};

template <typename Sink>
void AbslStringify(Sink& sink, const Sphere& sphere) {
  absl::Format(&sink, "Sphere(center: %s, radius: %f)", ToString(sphere.center),
               sphere.radius);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_SPHERE_H_
