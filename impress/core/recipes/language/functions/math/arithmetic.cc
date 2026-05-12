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
#include <cstddef>

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

bool FloatArrHasGreaterElement(const float* a, const float* b, int size) {
  for (size_t i = 0; i < size; ++i) {
    if (a[i] > b[i]) {
      return true;
    }
  }
  return false;
}

absl::StatusOr<recipe::Variable> Clamp(const recipe::Variable& value,
                                       const recipe::Variable& min,
                                       const recipe::Variable& max) {
  if (value.index() != min.index() || value.index() != max.index()) {
    return absl::InvalidArgumentError(
        "Clamp is not available for given types.");
  }

  switch (value.index()) {
    case Literal::kValue_IntValue:
      if (std::get<int>(min) > std::get<int>(max)) {
        break;
      }
      return std::clamp(std::get<int>(value), std::get<int>(min),
                        std::get<int>(max));

    case Literal::kValue_FloatValue:
      if (std::get<float>(min) > std::get<float>(max)) {
        break;
      }
      return std::clamp(std::get<float>(value), std::get<float>(min),
                        std::get<float>(max));

    case Literal::kValue_DoubleValue:
      if (std::get<double>(min) > std::get<double>(max)) {
        break;
      }
      return std::clamp(std::get<double>(value), std::get<double>(min),
                        std::get<double>(max));

    case Literal::kValue_Float2Value: {
      if (FloatArrHasGreaterElement(&std::get<float2>(min)[0],
                                    &std::get<float2>(max)[0], float2::SIZE)) {
        break;
      }
      return clamp(std::get<float2>(value), std::get<float2>(min),
                   std::get<float2>(max));
    }

    case Literal::kValue_Float3Value:
      if (FloatArrHasGreaterElement(&std::get<float3>(min)[0],
                                    &std::get<float3>(max)[0], float3::SIZE)) {
        break;
      }
      return clamp(std::get<float3>(value), std::get<float3>(min),
                   std::get<float3>(max));

    case Literal::kValue_Float4Value:
      if (FloatArrHasGreaterElement(&std::get<float4>(min)[0],
                                    &std::get<float4>(max)[0], float4::SIZE)) {
        break;
      }
      return clamp(std::get<float4>(value), std::get<float4>(min),
                   std::get<float4>(max));
    case Literal::kValue_Mat2fValue: {
      constexpr int size = mat2f::COL_SIZE * mat2f::ROW_SIZE;
      if (FloatArrHasGreaterElement(std::get<mat2f>(min).asArray(),
                                    std::get<mat2f>(max).asArray(), size)) {
        break;
      }
      mat2f result;
      for (int i = 0; i < size; ++i) {
        result[i / mat2f::COL_SIZE][i % mat2f::COL_SIZE] =
            clamp(std::get<mat2f>(value).asArray()[i],
                  std::get<mat2f>(min).asArray()[i],
                  std::get<mat2f>(max).asArray()[i]);
      }
      return result;
    }
    case Literal::kValue_Mat3fValue: {
      constexpr int size = mat3f::COL_SIZE * mat3f::ROW_SIZE;
      if (FloatArrHasGreaterElement(std::get<mat3f>(min).asArray(),
                                    std::get<mat3f>(max).asArray(), size)) {
        break;
      }
      mat3f result;
      for (int i = 0; i < size; ++i) {
        result[i / mat3f::COL_SIZE][i % mat3f::COL_SIZE] =
            clamp(std::get<mat3f>(value).asArray()[i],
                  std::get<mat3f>(min).asArray()[i],
                  std::get<mat3f>(max).asArray()[i]);
      }
      return result;
    }
    case Literal::kValue_Mat4fValue: {
      constexpr int size = mat4f::COL_SIZE * mat4f::ROW_SIZE;
      if (FloatArrHasGreaterElement(std::get<mat4f>(min).asArray(),
                                    std::get<mat4f>(max).asArray(), size)) {
        break;
      }
      mat4f result;
      for (int i = 0; i < size; ++i) {
        result[i / mat4f::COL_SIZE][i % mat4f::COL_SIZE] =
            clamp(std::get<mat4f>(value).asArray()[i],
                  std::get<mat4f>(min).asArray()[i],
                  std::get<mat4f>(max).asArray()[i]);
      }
      return result;
    }
    default:
      return absl::InvalidArgumentError(
          "Clamp is not available for given types.");
  }

  return absl::InvalidArgumentError(
      "One or more min values is greater than the corresponding max value.");
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
      return absl::InvalidArgumentError(
          "input must be a floating-point, floatN or matrix type.");
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
      return TransformMatrix(ceil, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(ceil, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(ceil, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point, floatN or matrix type.");
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
      return TransformMatrix(floor, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(floor, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(floor, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point, floatN or matrix type.");
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
      return TransformMatrix(round, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(round, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(round, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point, floatN or matrix type.");
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
          "input must be a floating-point, floatN or matrix type.");
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
          "input must be a floating-point, floatN or matrix type.");
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
      mat2f result;
      for (int i = 0; i < mat2f::COL_SIZE; ++i) {
        result[i] = mix(std::get<mat2f>(point1)[i], std::get<mat2f>(point2)[i],
                        std::get<mat2f>(coefficient)[i]);
      }
      return result;
    }
    case Literal::kValue_Mat3fValue: {
      mat3f result;
      for (int i = 0; i < mat3f::COL_SIZE; ++i) {
        result[i] = mix(std::get<mat3f>(point1)[i], std::get<mat3f>(point2)[i],
                        std::get<mat3f>(coefficient)[i]);
      }
      return result;
    }
    case Literal::kValue_Mat4fValue: {
      mat4f result;
      for (int i = 0; i < mat4f::COL_SIZE; ++i) {
        result[i] = mix(std::get<mat4f>(point1)[i], std::get<mat4f>(point2)[i],
                        std::get<mat4f>(coefficient)[i]);
      }
      return result;
    }
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point, floatN or matrix type.");
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
          "input must be a floating-point, floatN or matrix type.");
  }
}

}  // namespace

void RegisterMathArithmeticFunctions(BaseRecipeSystem* recipe_system) {
  recipe_system->RegisterFunction(
      "Clamp",
      [](const recipe::Variable& value, const recipe::Variable& min,
         const recipe::Variable& max) -> absl::StatusOr<recipe::Variable> {
        return Clamp(value, min, max);
      });
  recipe_system->RegisterFunction(
      "NegateValue",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return NegateValue(value);
      });
  recipe_system->RegisterFunction(
      "Ceil",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Ceil(value);
      });
  recipe_system->RegisterFunction(
      "Floor",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Floor(value);
      });
  recipe_system->RegisterFunction(
      "Round",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Round(value);
      });
  recipe_system->RegisterFunction(
      "Fraction",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Fraction(value);
      });
  recipe_system->RegisterFunction(
      "Saturate",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Saturate(value);
      });
  recipe_system->RegisterFunction(
      "Mix",
      [](const recipe::Variable& point1, const recipe::Variable& point2,
         const recipe::Variable& coefficient)
          -> absl::StatusOr<recipe::Variable> {
        return Mix(point1, point2, coefficient);
      });
  recipe_system->RegisterFunction(
      "Truncate",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Truncate(value);
      });
}

}  // namespace imp::recipe
