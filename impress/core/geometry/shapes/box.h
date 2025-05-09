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

#ifndef THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_BOX_H_
#define THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_BOX_H_

#include "filament/filament/include/filament/Box.h"
#include "core/math/vec.h"

namespace imp {
// An axis aligned 3D box represented by its center and half-extent. For the 2D
// version, please see the Rect class.
using Box = ::filament::Box;

// Increases the bounds of the given box to contain the given point.
template <typename T>
void ExtendBoundsToContainPoint(Box& box, const TVec3<T>& point) {
  box.set({fmin(box.getMin().x, point.x), fmin(box.getMin().y, point.y),
           fmin(box.getMin().z, point.z)},
          {fmax(box.getMax().x, point.x), fmax(box.getMax().y, point.y),
           fmax(box.getMax().z, point.z)});
}
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_BOX_H_
