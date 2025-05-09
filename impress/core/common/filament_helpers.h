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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_FILAMENT_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_FILAMENT_HELPERS_H_

#include "absl/status/statusor.h"
#include "core/geometry/shapes/box.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/math.h"

namespace imp {

// TODO: Migrate from sceneform, add tests.
// Apply a rigid transform to the provided bounds.  The output box is
// still axis-aligned; rotation is not applied to the extents.
Box TransformBounds(const Box& bounds, const mat4f& transform);
Rect TransformBounds(const Rect& bounds, const mat4f& transform);
Box TransformBounds(const Box& bounds, const mat4& transform);
Rect TransformBounds(const Rect& bounds, const mat4& transform);

// Return bounds whose min is set to FLT_MAX and whose max is set to -FLT_MAX.
Box NilBounds();

// compute <i, j, k> such that min + <i, j, k> * (max - min) == position.
absl::StatusOr<float3> ToBoundsFraction(const Box& bounds,
                                        const float3& position);

// Helper to decompose a 4x4 matrix into TRS (Translation, Rotation, Scale)
void Decompose(const mat4f& mat, float3* translation, quatf* rotation,
               float3* scale);

// Helper to compose a 4x4 matrix from TRS (Translation, Rotation, Scale)
imp::mat4f Compose(const float3& translation, const quatf& rotation,
                   const float3& scale);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_FILAMENT_HELPERS_H_
