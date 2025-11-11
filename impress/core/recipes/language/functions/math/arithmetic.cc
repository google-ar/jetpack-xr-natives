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

template <typename T>
bool VecHasGreaterElement(const T& a, const T& b) {
  for (size_t i = 0; i < a.size(); ++i) {
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
      if (VecHasGreaterElement(std::get<float2>(min), std::get<float2>(max))) {
        break;
      }
      return clamp(std::get<float2>(value), std::get<float2>(min),
                   std::get<float2>(max));
    }

    case Literal::kValue_Float3Value:
      if (VecHasGreaterElement(std::get<float3>(min), std::get<float3>(max))) {
        break;
      }
      return clamp(std::get<float3>(value), std::get<float3>(min),
                   std::get<float3>(max));

    case Literal::kValue_Float4Value:
      if (VecHasGreaterElement(std::get<float4>(min), std::get<float4>(max))) {
        break;
      }
      return clamp(std::get<float4>(value), std::get<float4>(min),
                   std::get<float4>(max));
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
