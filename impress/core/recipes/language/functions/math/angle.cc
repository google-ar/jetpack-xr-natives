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
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {

absl::StatusOr<Variable> Rad(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return ToRadians(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return ToRadians(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return TransformVector(ToRadians, std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return TransformVector(ToRadians, std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return TransformVector(ToRadians, std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(ToRadians, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(ToRadians, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(ToRadians, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<Variable> Deg(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return ToDegrees(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return ToDegrees(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return TransformVector(ToDegrees, std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return TransformVector(ToDegrees, std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return TransformVector(ToDegrees, std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(ToDegrees, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(ToDegrees, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(ToDegrees, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<Variable> Atan2(const recipe::Variable& y,
                               const recipe::Variable& x) {
  if (y.index() != x.index()) {
    return absl::InvalidArgumentError(
        "Atan2 must be used with the same type for both arguments.");
  }

  switch (y.index()) {
    case Literal::kValue_FloatValue:
      return std::atan2(std::get<float>(y), std::get<float>(x));
    case Literal::kValue_DoubleValue:
      return std::atan2(std::get<double>(y), std::get<double>(x));
    case Literal::kValue_Float2Value: {
      float2 value_y = std::get<float2>(y);
      float2 value_x = std::get<float2>(x);
      return float2(std::atan2(value_y.v[0], value_x.v[0]),
                    std::atan2(value_y.v[1], value_x.v[1]));
    }
    case Literal::kValue_Float3Value: {
      float3 value_y = std::get<float3>(y);
      float3 value_x = std::get<float3>(x);
      return float3(std::atan2(value_y.v[0], value_x.v[0]),
                    std::atan2(value_y.v[1], value_x.v[1]),
                    std::atan2(value_y.v[2], value_x.v[2]));
    }
    case Literal::kValue_Float4Value: {
      float4 value_y = std::get<float4>(y);
      float4 value_x = std::get<float4>(x);
      return float4(std::atan2(value_y.v[0], value_x.v[0]),
                    std::atan2(value_y.v[1], value_x.v[1]),
                    std::atan2(value_y.v[2], value_x.v[2]),
                    std::atan2(value_y.v[3], value_x.v[3]));
    }
    case Literal::kValue_Mat2fValue: {
      mat2f value_y = std::get<mat2f>(y);
      mat2f value_x = std::get<mat2f>(x);
      return mat2f(std::atan2(value_y[0][0], value_x[0][0]),
                   std::atan2(value_y[0][1], value_x[0][1]),
                   std::atan2(value_y[1][0], value_x[1][0]),
                   std::atan2(value_y[1][1], value_x[1][1]));
    }
    case Literal::kValue_Mat3fValue: {
      mat3f value_y = std::get<mat3f>(y);
      mat3f value_x = std::get<mat3f>(x);
      return mat3f(std::atan2(value_y[0][0], value_x[0][0]),
                   std::atan2(value_y[0][1], value_x[0][1]),
                   std::atan2(value_y[0][2], value_x[0][2]),
                   std::atan2(value_y[1][0], value_x[1][0]),
                   std::atan2(value_y[1][1], value_x[1][1]),
                   std::atan2(value_y[1][2], value_x[1][2]),
                   std::atan2(value_y[2][0], value_x[2][0]),
                   std::atan2(value_y[2][1], value_x[2][1]),
                   std::atan2(value_y[2][2], value_x[2][2]));
    }
    case Literal::kValue_Mat4fValue: {
      mat4f value_y = std::get<mat4f>(y);
      mat4f value_x = std::get<mat4f>(x);
      return mat4f(std::atan2(value_y[0][0], value_x[0][0]),
                   std::atan2(value_y[0][1], value_x[0][1]),
                   std::atan2(value_y[0][2], value_x[0][2]),
                   std::atan2(value_y[0][3], value_x[0][3]),
                   std::atan2(value_y[1][0], value_x[1][0]),
                   std::atan2(value_y[1][1], value_x[1][1]),
                   std::atan2(value_y[1][2], value_x[1][2]),
                   std::atan2(value_y[1][3], value_x[1][3]),
                   std::atan2(value_y[2][0], value_x[2][0]),
                   std::atan2(value_y[2][1], value_x[2][1]),
                   std::atan2(value_y[2][2], value_x[2][2]),
                   std::atan2(value_y[2][3], value_x[2][3]),
                   std::atan2(value_y[3][0], value_x[3][0]),
                   std::atan2(value_y[3][1], value_x[3][1]),
                   std::atan2(value_y[3][2], value_x[3][2]),
                   std::atan2(value_y[3][3], value_x[3][3]));
    }
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}
}  // namespace

void RegisterMathAngleFunctions(BaseRecipeSystem* recipe_system) {
  recipe_system->RegisterFunction(
      "Rad", [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Rad(value);
      });

  recipe_system->RegisterFunction(
      "Deg", [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Deg(value);
      });

  recipe_system->RegisterFunction(
      "Atan2",
      [](recipe::Variable y, recipe::Variable x)
          -> absl::StatusOr<recipe::Variable> { return Atan2(y, x); });
}

}  // namespace imp::recipe
