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

#include <algorithm>
#include <cmath>

#include "absl/strings/string_view.h"
#include "core/math/almost_equal.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {

float4 Float4QuatMultiply(float4 a, float4 b) {
  quatf result = quatf(a) * quatf(b);
  return float4(result.x, result.y, result.z, result.w);
}

float4 Float4QuatFromAxisAngle(float3 a, float b) {
  quatf result = quatf::fromAxisAngle(a, b);
  return float4(result.x, result.y, result.z, result.w);
}

float Float4QuatAngleBetween(float4 a, float4 b) {
  return 2.f * std::acos(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
}

float4 Float4QuatConjugate(float4 a) { return float4(-a.x, -a.y, -a.z, a.w); }

recipe::Variables Float4QuatToAxisAngle(float4 a) {
  recipe::Variables variables;

  // Clamp the w component so we don't get a NAN from a little floating point
  // imprecision in previous calculations.
  float w = std::clamp(a.w, -1.f, 1.f);

  // If we have an angle of zero (e.g. arcos(1)) then the axis is undefined
  // and we return the x-axis basis vector as per the spec.
  if (AlmostEqual(std::fabs(w), 1.f)) {
    variables["axis"] = float3(1.f, 0.f, 0.f);
    variables["angle"] = 0.f;
  } else {
    float angle = 2.f * std::acos(w);
    float axis_length = std::sin(angle / 2.f);
    float3 axis = float3(a.x, a.y, a.z) / axis_length;
    variables["axis"] = axis;
    variables["angle"] = angle;
  }
  return variables;
}

float4 Float4QuatFromDirections(float3 a, float3 b) {
  quatf result = quatf::fromDirectedRotation(a, b);
  return float4(result.x, result.y, result.z, result.w);
}

float4 Float4QuatSlerp(float4 p, float4 q, float t) {
  quatf unit_p = normalize(quatf(p));
  quatf unit_q = normalize(quatf(q));
  quatf result = slerp(unit_p, unit_q, t);
  return float4(result.x, result.y, result.z, result.w);
}

float4 Float4QuatFromUpForward(float3 up, float3 forward) {
  float3 r(forward);
  float3 y(up);
  float3 s;

  // Check if y and r are collinear.
  // We use a threshold of 0.999 (similar to filament's dot_tolerance).
  if (std::abs(dot(y, r)) > 0.999) {
    // y and r are collinear.
    // Let s be perpendicular to r.
    // We can pick an arbitrary axis to cross with r.
    float3 arbitrary =
        std::abs(r.z) < 0.999 ? float3(0.0, 0.0, 1.0) : float3(1.0, 0.0, 0.0);
    s = normalize(cross(arbitrary, r));
  } else {
    s = normalize(cross(y, r));
  }

  float3 t = cross(r, s);

  // Construct rotation matrix M = [s, t, r] (columns).
  mat3f M(s, t, r);

  quatf result = M.toQuaternion();
  return float4(result.x, result.y, result.z, result.w);
}

}  // namespace

void RegisterMathQuaternionFunctions(BaseRecipeSystem* recipe_system) {
  recipe_system->RegisterFunction(
      "Float4QuatMultiply",
      [](float4 a, float4 b) -> float4 { return Float4QuatMultiply(a, b); });

  recipe_system->RegisterFunction("Float4QuatFromAxisAngle",
                                  [](float3 a, float b) -> float4 {
                                    return Float4QuatFromAxisAngle(a, b);
                                  });

  recipe_system->RegisterFunction(
      "Float4QuatAngleBetween",
      [](float4 a, float4 b) -> float { return Float4QuatAngleBetween(a, b); });

  recipe_system->RegisterFunction(
      "Float4QuatConjugate",
      [](float4 a) -> float4 { return Float4QuatConjugate(a); });

  recipe_system->RegisterFunction("Float4QuatToAxisAngle", [](float4 a) {
    return Float4QuatToAxisAngle(a);
  });

  recipe_system->RegisterFunction("Float4QuatFromDirections",
                                  [](float3 a, float3 b) -> float4 {
                                    return Float4QuatFromDirections(a, b);
                                  });

  recipe_system->RegisterFunction(
      "Float4QuatSlerp",
      [](const float4 a, const float4 b, const float c) -> float4 {
        return Float4QuatSlerp(a, b, c);
      });

  recipe_system->RegisterFunction("Float4QuatFromUpForward",
                                  [](float3 up, float3 forward) -> float4 {
                                    return Float4QuatFromUpForward(up, forward);
                                  });
}

}  // namespace imp::recipe
