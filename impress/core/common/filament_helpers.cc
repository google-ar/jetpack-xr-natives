// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/common/filament_helpers.h"

#include "core/geometry/geometry_helper.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/math.h"

namespace imp {

using ::filament::math::float3;
using ::filament::math::mat3f;
using ::filament::math::mat4f;
using ::filament::math::quatf;

filament::Box TransformBounds(const filament::Box& bounds,
                              const mat4f& transform) {
  // rigidTransform isn't exposed out of libfilament-jni :-(
  const mat3f rotation(transform.upperLeft());
  return {rotation * bounds.center + transform[3].xyz,
          abs(rotation) * bounds.halfExtent};
}

Rect TransformBounds(const Rect& bounds, const mat4f& transform) {
  return ToRect(TransformBounds(ToBox(bounds), transform));
}

filament::Box TransformBounds(const filament::Box& bounds,
                              const mat4& transform) {
  // TODO: (broken link) - Precision loss due to single precision bounds.
  return TransformBounds(bounds, mat4f(transform));
}

Rect TransformBounds(const Rect& bounds, const mat4& transform) {
  return ToRect(TransformBounds(ToBox(bounds), transform));
}

filament::Box NilBounds() {
  using float_limits = std::numeric_limits<float>;
  return filament::Box{}.set({float_limits::max()}, {float_limits::lowest()});
}

absl::StatusOr<float3> ToBoundsFraction(
    const filament::Box& bounds, const filament::math::float3& position) {
  constexpr float kEpsilon = 1e-5f;
  float3 min = bounds.getMin();
  float3 max = bounds.getMax();
  float3 delta = max - min;
  // Since we're performing divisions, ensure we have legal bounds.
  if (std::min(std::min(delta.x, delta.y), delta.z) < kEpsilon)
    return absl::InvalidArgumentError("Invalid Bounds");

  return (position - min) / delta;
}

void Decompose(const mat4f& mat, float3* translation, quatf* rotation,
               float3* scale) {
  assert(translation && rotation && scale);
  auto trs = Transform<float>(mat);
  *translation = trs.translation;
  *rotation = trs.rotation;
  *scale = trs.scale;
}

mat4f Compose(const float3& translation, const quatf& rotation,
              const float3& scale) {
  return Transform<float>(translation, rotation, scale).AsMat4();
}

}  // namespace imp
