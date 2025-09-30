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
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {

float4 Float4QuatMultiply(const float4 a, const float4 b) {
  quatf result = quatf(a) * quatf(b);
  return float4(result.x, result.y, result.z, result.w);
}

float4 Float4QuatFromAxisAngle(const float3 a, const float b) {
  quatf result = quatf::fromAxisAngle(a, b);
  return float4(result.x, result.y, result.z, result.w);
}

float Float4QuatAngleBetween(const float4 a, const float4 b) {
  return 2.f * std::acos(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
}

float4 Float4QuatConjugate(const float4 a) {
  return float4(-a.x, -a.y, -a.z, a.w);
}

recipe::Variables Float4QuatToAxisAngle(const float4 a) {
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

float4 Float4QuatFromDirections(const float3 a, const float3 b) {
  quatf result = quatf::fromDirectedRotation(a, b);
  return float4(result.x, result.y, result.z, result.w);
}

}  // namespace

void RegisterMathQuaternionFunctions(BaseRecipeSystem* recipe_system) {
  recipe_system->RegisterFunction("Float4QuatMultiply",
                                  [](const float4 a, const float4 b) -> float4 {
                                    return Float4QuatMultiply(a, b);
                                  });

  recipe_system->RegisterFunction("Float4QuatFromAxisAngle",
                                  [](const float3 a, const float b) -> float4 {
                                    return Float4QuatFromAxisAngle(a, b);
                                  });

  recipe_system->RegisterFunction("Float4QuatAngleBetween",
                                  [](const float4 a, const float4 b) -> float {
                                    return Float4QuatAngleBetween(a, b);
                                  });

  recipe_system->RegisterFunction(
      "Float4QuatConjugate",
      [](const float4 a) -> float4 { return Float4QuatConjugate(a); });

  recipe_system->RegisterFunction("Float4QuatToAxisAngle", [](float4 a) {
    return Float4QuatToAxisAngle(a);
  });

  recipe_system->RegisterFunction("Float4QuatFromDirections",
                                  [](const float3 a, const float3 b) -> float4 {
                                    return Float4QuatFromDirections(a, b);
                                  });
}

}  // namespace imp::recipe
