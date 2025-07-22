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

#ifndef THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_CYLINDER_H_
#define THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_CYLINDER_H_

#include "core/math/vec.h"

namespace imp {

/* Represent a cylinder. Default bottom plane is the xz plane, with Y-up as the
 * axis direction, 1 as the height of cylinder and the radius of the bottom
 * cycle.
 * center: the center of bottom plane.
 * up: axis direction.
 * length: length on axis direction, commonly known as "height of a cylinder".
 * forward: for axis to the 0 degree line on side surface.
 * radius: radius of the cylinder.
 */
template <typename T>
struct GenericCylinder {
  GenericCylinder()
      : center(TVec3<T>(0, 0, 0)),
        up(TVec3<T>(0, 1, 0)),
        length(1),
        forward(TVec3<T>(1, 0, 0)),
        radius(0.1) {}

  GenericCylinder(TVec3<T> center, TVec3<T> up, T length, TVec3<T> forward,
                  T radius)
      : center(center),
        up(up),
        length(length),
        forward(forward),
        radius(radius) {}

  TVec3<T> center;
  TVec3<T> up;
  T length;
  TVec3<T> forward;
  T radius;
};

// Represents a cylinder with bottom ring center on xz plane and height on Y-up.
struct Cylinder {
  float3 base;
  float radius;
  float height;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GEOMETRY_SHAPES_CYLINDER_H_
