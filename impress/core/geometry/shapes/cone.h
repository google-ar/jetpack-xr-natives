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

#ifndef THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_CONE_H_
#define THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_CONE_H_

#include "absl/strings/str_format.h"
#include "core/math/vec.h"

namespace imp {

// Represents a cone with height parallel to the Y-up axis
struct Cone {
  float3 base;
  float radius;
  float height;

  template <typename Sink>
  friend void AbslStringify(Sink& sink, const Cone& cone);
};

template <typename Sink>
void AbslStringify(Sink& sink, const Cone& cone) {
  absl::Format(&sink, "Cone(base: %s, radius: %f, height: %f)",
               ToString(cone.base), cone.radius, cone.height);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_CONE_H_
