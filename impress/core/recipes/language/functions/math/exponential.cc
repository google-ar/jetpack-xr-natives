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
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {

absl::StatusOr<recipe::Variable> Exp(const recipe::Variable value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::exp(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::exp(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return exp(std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return exp(std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return exp(std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(std::exp, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(std::exp, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(std::exp, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<recipe::Variable> Log2(const recipe::Variable value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::log2(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::log2(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return log2(std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return log2(std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return log2(std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(std::log2, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(std::log2, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(std::log2, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<recipe::Variable> Log10(const recipe::Variable value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::log10(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::log10(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return log10(std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return log10(std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return log10(std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(std::log10, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(std::log10, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(std::log10, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<recipe::Variable> CubeRoot(const recipe::Variable value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::cbrt(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::cbrt(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return cbrt(std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return cbrt(std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return cbrt(std::get<float4>(value));
    case Literal::kValue_Mat2fValue:
      return TransformMatrix(std::cbrt, std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return TransformMatrix(std::cbrt, std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return TransformMatrix(std::cbrt, std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<recipe::Variable> Pow(const recipe::Variable base,
                                     const recipe::Variable exponent) {
  if (base.index() != exponent.index()) {
    return absl::InvalidArgumentError(
        "base and exponent must be of the same type.");
  }

  switch (base.index()) {
    case Literal::kValue_FloatValue:
      return std::pow(std::get<float>(base), std::get<float>(exponent));
    case Literal::kValue_DoubleValue:
      return std::pow(std::get<double>(base), std::get<double>(exponent));
    case Literal::kValue_Float2Value:
      return pow(std::get<float2>(base), std::get<float2>(exponent));
    case Literal::kValue_Float3Value:
      return pow(std::get<float3>(base), std::get<float3>(exponent));
    case Literal::kValue_Float4Value:
      return pow(std::get<float4>(base), std::get<float4>(exponent));
    case Literal::kValue_Mat2fValue: {
      mat2f base_value = std::get<mat2f>(base);
      mat2f exponent_value = std::get<mat2f>(exponent);
      return mat2f(std::pow(base_value[0][0], exponent_value[0][0]),
                   std::pow(base_value[0][1], exponent_value[0][1]),
                   std::pow(base_value[1][0], exponent_value[1][0]),
                   std::pow(base_value[1][1], exponent_value[1][1]));
    }
    case Literal::kValue_Mat3fValue: {
      mat3f base_value = std::get<mat3f>(base);
      mat3f exponent_value = std::get<mat3f>(exponent);
      return mat3f(std::pow(base_value[0][0], exponent_value[0][0]),
                   std::pow(base_value[0][1], exponent_value[0][1]),
                   std::pow(base_value[0][2], exponent_value[0][2]),
                   std::pow(base_value[1][0], exponent_value[1][0]),
                   std::pow(base_value[1][1], exponent_value[1][1]),
                   std::pow(base_value[1][2], exponent_value[1][2]),
                   std::pow(base_value[2][0], exponent_value[2][0]),
                   std::pow(base_value[2][1], exponent_value[2][1]),
                   std::pow(base_value[2][2], exponent_value[2][2]));
    }
    case Literal::kValue_Mat4fValue: {
      mat4f base_value = std::get<mat4f>(base);
      mat4f exponent_value = std::get<mat4f>(exponent);
      return mat4f(std::pow(base_value[0][0], exponent_value[0][0]),
                   std::pow(base_value[0][1], exponent_value[0][1]),
                   std::pow(base_value[0][2], exponent_value[0][2]),
                   std::pow(base_value[0][3], exponent_value[0][3]),
                   std::pow(base_value[1][0], exponent_value[1][0]),
                   std::pow(base_value[1][1], exponent_value[1][1]),
                   std::pow(base_value[1][2], exponent_value[1][2]),
                   std::pow(base_value[1][3], exponent_value[1][3]),
                   std::pow(base_value[2][0], exponent_value[2][0]),
                   std::pow(base_value[2][1], exponent_value[2][1]),
                   std::pow(base_value[2][2], exponent_value[2][2]),
                   std::pow(base_value[2][3], exponent_value[2][3]),
                   std::pow(base_value[3][0], exponent_value[3][0]),
                   std::pow(base_value[3][1], exponent_value[3][1]),
                   std::pow(base_value[3][2], exponent_value[3][2]),
                   std::pow(base_value[3][3], exponent_value[3][3]));
    }
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

}  // namespace

void RegisterMathExponentialFunctions(BaseRecipeSystem* recipe_system) {
  recipe_system->RegisterFunction(
      "Exp", [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Exp(value);
      });

  recipe_system->RegisterFunction(
      "Log2", [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Log2(value);
      });

  recipe_system->RegisterFunction(
      "Log10", [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Log10(value);
      });

  recipe_system->RegisterFunction(
      "CubeRoot",
      [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return CubeRoot(value);
      });

  recipe_system->RegisterFunction(
      "Pow",
      [](recipe::Variable base, recipe::Variable exponent)
          -> absl::StatusOr<recipe::Variable> { return Pow(base, exponent); });
}

}  // namespace imp::recipe
