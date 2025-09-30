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

#include <cmath>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {

recipe::Variables Extract2(const float2 input) {
  recipe::Variables variables;
  variables["0"] = input.x;
  variables["1"] = input.y;
  return variables;
}

recipe::Variables Extract3(const float3 input) {
  recipe::Variables variables;
  variables["0"] = input.x;
  variables["1"] = input.y;
  variables["2"] = input.z;
  return variables;
}

recipe::Variables Extract4(const float4 input) {
  recipe::Variables variables;
  variables["0"] = input.x;
  variables["1"] = input.y;
  variables["2"] = input.z;
  variables["3"] = input.w;
  return variables;
}

absl::StatusOr<float> GetVectorLength(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_Float2Value:
      return length(std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return length(std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return length(std::get<float4>(value));
    default:
      return absl::InvalidArgumentError("input must be a floatN type.");
  }
}

float2 Rotate2d(const float2 vector, float angle) {
  auto rotationMatrix =
      mat2f(float2(cos(angle), sin(angle)), float2(-sin(angle), cos(angle)));
  return rotationMatrix * vector;
}

float3 Rotate3d(const float3 origVector, const float3 axis, float angle) {
  quatf rotation = quatf::fromAxisAngle(axis, angle);
  return rotation * origVector;
}

absl::StatusOr<recipe::Variable> Transform(const recipe::Variable& vector,
                                           const recipe::Variable& matrix) {
  switch (vector.index()) {
    case Literal::kValue_Float2Value:
      if (matrix.index() != Literal::kValue_Mat2fValue) {
        return absl::InvalidArgumentError("argument 2 must be a Mat2F value.");
      }
      return (float2)(std::get<mat2f>(matrix) * std::get<float2>(vector));
    case Literal::kValue_Float3Value:
      if (matrix.index() != Literal::kValue_Mat3fValue) {
        return absl::InvalidArgumentError("argument 2 must be a Mat3F value.");
      }
      return (float3)(std::get<mat3f>(matrix) * std::get<float3>(vector));
    case Literal::kValue_Float4Value:
      if (matrix.index() != Literal::kValue_Mat4fValue) {
        return absl::InvalidArgumentError("argument 2 must be a Mat4F value.");
      }
      return (float4)(std::get<mat4f>(matrix) * std::get<float4>(vector));
    default:
      return absl::InvalidArgumentError("argument 1 must be a FloatN type.");
  }
}

}  // namespace

void RegisterMathVectorFunctions(BaseRecipeSystem* recipe_system) {
  recipe_system->RegisterFunction(
      "Combine2", [](float a, float b) { return float2(a, b); });

  recipe_system->RegisterFunction(
      "Combine3", [](float a, float b, float c) { return float3(a, b, c); });

  recipe_system->RegisterFunction(
      "Combine4",
      [](float a, float b, float c, float d) { return float4(a, b, c, d); });

  recipe_system->RegisterFunction("Extract2",
                                  [](float2 input) { return Extract2(input); });

  recipe_system->RegisterFunction("Extract3",
                                  [](float3 input) { return Extract3(input); });

  recipe_system->RegisterFunction("Extract4",
                                  [](float4 input) { return Extract4(input); });

  recipe_system->RegisterFunction(
      "GetVectorLength", [](recipe::Variable value) -> absl::StatusOr<float> {
        return GetVectorLength(value);
      });

  recipe_system->RegisterFunction("Rotate2d", [](float2 vector, float angle) {
    return Rotate2d(vector, angle);
  });

  recipe_system->RegisterFunction("Rotate3d",
                                  [](float3 vector, float3 axis, float angle) {
                                    return Rotate3d(vector, axis, angle);
                                  });

  recipe_system->RegisterFunction(
      "Transform",
      [](recipe::Variable vector,
         recipe::Variable matrix) -> absl::StatusOr<recipe::Variable> {
        return Transform(vector, matrix);
      });
}

}  // namespace imp::recipe
