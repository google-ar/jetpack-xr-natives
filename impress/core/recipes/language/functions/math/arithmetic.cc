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

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {

absl::StatusOr<recipe::Variable> Clamp(const recipe::Variable& value,
                                       const recipe::Variable& min,
                                       const recipe::Variable& max) {
  if (value.index() != min.index() || value.index() != max.index()) {
    return absl::InvalidArgumentError(
        "Clamp is not available for given types.");
  }

  switch (value.index()) {
    case Literal::kValue_IntValue:
      return std::clamp(std::get<int>(value), std::get<int>(min),
                        std::get<int>(max));
    case Literal::kValue_FloatValue:
      return std::clamp(std::get<float>(value), std::get<float>(min),
                        std::get<float>(max));
    case Literal::kValue_DoubleValue:
      return std::clamp(std::get<double>(value), std::get<double>(min),
                        std::get<double>(max));
    case Literal::kValue_Float2Value:
      return clamp(std::get<float2>(value), std::get<float2>(min),
                   std::get<float2>(max));
    case Literal::kValue_Float3Value:
      return clamp(std::get<float3>(value), std::get<float3>(min),
                   std::get<float3>(max));
    case Literal::kValue_Float4Value:
      return clamp(std::get<float4>(value), std::get<float4>(min),
                   std::get<float4>(max));
    case Literal::kValue_Mat2fValue: {
      mat2f result{0.f};
      for (int i = 0; i < result.size(); ++i) {
        result[i] = clamp(std::get<mat2f>(value)[i], std::get<mat2f>(min)[i],
                          std::get<mat2f>(max)[i]);
      }
      return result;
    }
    case Literal::kValue_Mat3fValue: {
      mat3f result{0.f};
      for (int i = 0; i < result.size(); ++i) {
        result[i] = clamp(std::get<mat3f>(value)[i], std::get<mat3f>(min)[i],
                          std::get<mat3f>(max)[i]);
      }
      return result;
    }
    case Literal::kValue_Mat4fValue: {
      mat4f result{0.f};
      for (int i = 0; i < result.size(); ++i) {
        result[i] = clamp(std::get<mat4f>(value)[i], std::get<mat4f>(min)[i],
                          std::get<mat4f>(max)[i]);
      }
      return result;
    }
    default:
      return absl::InvalidArgumentError(
          "Clamp is not available for given types.");
  }
}

absl::StatusOr<recipe::Variable> NegateValue(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_IntValue:
      return -std::get<int>(value);
    case Literal::kValue_DoubleValue:
      return -std::get<double>(value);
    case Literal::kValue_FloatValue:
      return -std::get<float>(value);
    case Literal::kValue_Float2Value:
      return -std::get<float2>(value);
    case Literal::kValue_Float3Value:
      return -std::get<float3>(value);
    case Literal::kValue_Float4Value:
      return -std::get<float4>(value);
    case Literal::kValue_Mat2fValue:
      return -std::get<mat2f>(value);
    case Literal::kValue_Mat3fValue:
      return -std::get<mat3f>(value);
    case Literal::kValue_Mat4fValue:
      return -std::get<mat4f>(value);
    default:
      return absl::InvalidArgumentError("input must be an arithmetic type.");
  }
}

absl::StatusOr<recipe::Variable> Ceil(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::ceil(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::ceil(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return ceil(std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return ceil(std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return ceil(std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(std::ceil, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(std::ceil, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(std::ceil, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError("input must be an arithmetic type.");
  }
}

absl::StatusOr<recipe::Variable> Floor(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::floor(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::floor(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return floor(std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return floor(std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return floor(std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(std::floor, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(std::floor, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(std::floor, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<recipe::Variable> Round(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::round(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::round(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return round(std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return round(std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return round(std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(std::round, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(std::round, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(std::round, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<recipe::Variable> Fraction(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue: {
      const float v = std::get<float>(value);
      return v - std::floor(v);
    }
    case Literal::kValue_DoubleValue: {
      const double v = std::get<double>(value);
      return v - std::floor(v);
    }
    case Literal::kValue_Float2Value:
      return TransformVector(fraction, std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return TransformVector(fraction, std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return TransformVector(fraction, std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(fraction, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(fraction, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(fraction, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<recipe::Variable> Saturate(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return saturate(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return saturate(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return saturate(std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return saturate(std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return saturate(std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(saturate, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(saturate, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(saturate, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<recipe::Variable> Mix(const recipe::Variable& point1,
                                     const recipe::Variable& point2,
                                     const recipe::Variable& coefficient) {
  if (point1.index() != point2.index() ||
      point1.index() != coefficient.index()) {
    return absl::InvalidArgumentError(
        "all arguments must be of the same type.");
  }

  switch (point1.index()) {
    case Literal::kValue_FloatValue:
      return mix(std::get<float>(point1), std::get<float>(point2),
                 std::get<float>(coefficient));
    case Literal::kValue_DoubleValue:
      return mix(std::get<double>(point1), std::get<double>(point2),
                 std::get<double>(coefficient));
    case Literal::kValue_Float2Value:
      return mix(std::get<float2>(point1), std::get<float2>(point2),
                 std::get<float2>(coefficient));
    case Literal::kValue_Float3Value:
      return mix(std::get<float3>(point1), std::get<float3>(point2),
                 std::get<float3>(coefficient));
    case Literal::kValue_Float4Value:
      return mix(std::get<float4>(point1), std::get<float4>(point2),
                 std::get<float4>(coefficient));
    case Literal::kValue_Mat2fValue: {
      mat2f point1_value = std::get<mat2f>(point1);
      mat2f point2_value = std::get<mat2f>(point2);
      mat2f coefficient_value = std::get<mat2f>(coefficient);
      return mat2f(
          mix(point1_value[0][0], point2_value[0][0], coefficient_value[0][0]),
          mix(point1_value[0][1], point2_value[0][1], coefficient_value[0][1]),
          mix(point1_value[1][0], point2_value[1][0], coefficient_value[1][0]),
          mix(point1_value[1][1], point2_value[1][1], coefficient_value[1][1]));
    }
    case Literal::kValue_Mat3fValue: {
      mat3f point1_value = std::get<mat3f>(point1);
      mat3f point2_value = std::get<mat3f>(point2);
      mat3f coefficient_value = std::get<mat3f>(coefficient);
      return mat3f(
          mix(point1_value[0][0], point2_value[0][0], coefficient_value[0][0]),
          mix(point1_value[0][1], point2_value[0][1], coefficient_value[0][1]),
          mix(point1_value[0][2], point2_value[0][2], coefficient_value[0][2]),
          mix(point1_value[1][0], point2_value[1][0], coefficient_value[1][0]),
          mix(point1_value[1][1], point2_value[1][1], coefficient_value[1][1]),
          mix(point1_value[1][2], point2_value[1][2], coefficient_value[1][2]),
          mix(point1_value[2][0], point2_value[2][0], coefficient_value[2][0]),
          mix(point1_value[2][1], point2_value[2][1], coefficient_value[2][1]),
          mix(point1_value[2][2], point2_value[2][2], coefficient_value[2][2]));
    }
    case Literal::kValue_Mat4fValue: {
      mat4f point1_value = std::get<mat4f>(point1);
      mat4f point2_value = std::get<mat4f>(point2);
      mat4f coefficient_value = std::get<mat4f>(coefficient);
      return mat4f(
          mix(point1_value[0][0], point2_value[0][0], coefficient_value[0][0]),
          mix(point1_value[0][1], point2_value[0][1], coefficient_value[0][1]),
          mix(point1_value[0][2], point2_value[0][2], coefficient_value[0][2]),
          mix(point1_value[0][3], point2_value[0][3], coefficient_value[0][3]),
          mix(point1_value[1][0], point2_value[1][0], coefficient_value[1][0]),
          mix(point1_value[1][1], point2_value[1][1], coefficient_value[1][1]),
          mix(point1_value[1][2], point2_value[1][2], coefficient_value[1][2]),
          mix(point1_value[1][3], point2_value[1][3], coefficient_value[1][3]),
          mix(point1_value[2][0], point2_value[2][0], coefficient_value[2][0]),
          mix(point1_value[2][1], point2_value[2][1], coefficient_value[2][1]),
          mix(point1_value[2][2], point2_value[2][2], coefficient_value[2][2]),
          mix(point1_value[2][3], point2_value[2][3], coefficient_value[2][3]),
          mix(point1_value[3][0], point2_value[3][0], coefficient_value[3][0]),
          mix(point1_value[3][1], point2_value[3][1], coefficient_value[3][1]),
          mix(point1_value[3][2], point2_value[3][2], coefficient_value[3][2]),
          mix(point1_value[3][3], point2_value[3][3], coefficient_value[3][3]));
    }
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<recipe::Variable> Truncate(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::trunc(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::trunc(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return TransformVector(std::trunc, std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return TransformVector(std::trunc, std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return TransformVector(std::trunc, std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(std::trunc, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(std::trunc, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(std::trunc, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

}  // namespace

void RegisterMathArithmeticFunctions(BaseRecipeSystem* recipe_system) {
  recipe_system->RegisterFunction(
      "Clamp",
      [](recipe::Variable value, recipe::Variable min,
         recipe::Variable max) -> absl::StatusOr<recipe::Variable> {
        return Clamp(value, min, max);
      });
  recipe_system->RegisterFunction(
      "NegateValue",
      [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return NegateValue(value);
      });
  recipe_system->RegisterFunction(
      "Ceil", [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Ceil(value);
      });
  recipe_system->RegisterFunction(
      "Floor", [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Floor(value);
      });
  recipe_system->RegisterFunction(
      "Round", [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Round(value);
      });
  recipe_system->RegisterFunction(
      "Fraction",
      [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Fraction(value);
      });
  recipe_system->RegisterFunction(
      "Saturate",
      [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Saturate(value);
      });
  recipe_system->RegisterFunction(
      "Mix",
      [](recipe::Variable point1, recipe::Variable point2,
         recipe::Variable coefficient) -> absl::StatusOr<recipe::Variable> {
        return Mix(point1, point2, coefficient);
      });
  recipe_system->RegisterFunction(
      "Truncate",
      [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Truncate(value);
      });
}

}  // namespace imp::recipe
