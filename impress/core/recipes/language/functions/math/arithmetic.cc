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
#include "core/math/vec.h"
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
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
    default:
      return absl::InvalidArgumentError(
          "Clamp is not available for given types.");
  }
}

absl::StatusOr<recipe::Variable> NegateValue(const recipe::Variable& value) {
  switch (value.index()) {
    case VariableType::INT:
      return -std::get<int>(value);
    case VariableType::DOUBLE:
      return -std::get<double>(value);
    case VariableType::FLOAT:
      return -std::get<float>(value);
    case VariableType::FLOAT2:
      return -std::get<float2>(value);
    case VariableType::FLOAT3:
      return -std::get<float3>(value);
    case VariableType::FLOAT4:
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

absl::StatusOr<recipe::Variable> Saturate(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::min(std::max(std::get<float>(value), 0.0f), 1.0f);
    case Literal::kValue_DoubleValue:
      return std::min(std::max(std::get<double>(value), 0.0), 1.0);
    case Literal::kValue_Float2Value:
      return min(max(std::get<float2>(value), kZero2), kOne2);
    case Literal::kValue_Float3Value:
      return min(max(std::get<float3>(value), kZero3), kOne3);
    case Literal::kValue_Float4Value:
      return min(max(std::get<float4>(value), kZero4), kOne4);
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
      return (1.0f - std::get<float>(coefficient)) * std::get<float>(point1) +
             std::get<float>(coefficient) * std::get<float>(point2);
    case Literal::kValue_DoubleValue:
      return (1.0 - std::get<double>(coefficient)) * std::get<double>(point1) +
             std::get<double>(coefficient) * std::get<double>(point2);
    case Literal::kValue_Float2Value:
      return (kOne2 - std::get<float2>(coefficient)) *
                 std::get<float2>(point1) +
             std::get<float2>(coefficient) * std::get<float2>(point2);
    case Literal::kValue_Float3Value:
      return (kOne3 - std::get<float3>(coefficient)) *
                 std::get<float3>(point1) +
             std::get<float3>(coefficient) * std::get<float3>(point2);
    case Literal::kValue_Float4Value:
      return (kOne4 - std::get<float4>(coefficient)) *
                 std::get<float4>(point1) +
             std::get<float4>(coefficient) * std::get<float4>(point2);
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
}

}  // namespace imp::recipe
