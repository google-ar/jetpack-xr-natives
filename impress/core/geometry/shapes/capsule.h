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

#ifndef THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_CAPSULE_H_
#define THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_CAPSULE_H_

#include "absl/strings/str_format.h"
#include "core/math/vec.h"

namespace imp {

// Represents a capsule, which is a cylinder with two hemispheres at each end.
// The cylinder's center line is parallel to the Y-axis, from center-height/2 to
// center+height/2.
struct Capsule {
  float3 center;
  float height;
  float radius;

  template <typename Sink>
  friend void AbslStringify(Sink& sink, const Capsule& capsule);
};

template <typename Sink>
void AbslStringify(Sink& sink, const Capsule& capsule) {
  absl::Format(&sink,
               "Capsule(center: %s, middle cylinder height: %f, radius: %f)",
               ToString(capsule.center), capsule.height, capsule.radius);
}
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_CAPSULE_H_
