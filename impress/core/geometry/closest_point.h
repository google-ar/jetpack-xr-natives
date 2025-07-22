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

#ifndef THIRD_PARTY_IMPRESS_CORE_GEOMETRY_CLOSEST_POINT_H_
#define THIRD_PARTY_IMPRESS_CORE_GEOMETRY_CLOSEST_POINT_H_

#include "core/collision/ray.h"
#include "core/geometry/shapes/box.h"
#include "core/math/vec.h"

namespace imp {

// Finds the closest point on the box to the point.
float3 ClosestPointOnBoxToPoint(const Box& box, const float3& point);

// Finds the closest point on the box to the ray.
float3 ClosestPointOnBoxToRay(const Box& box, const Ray& ray);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GEOMETRY_CLOSEST_POINT_H_
