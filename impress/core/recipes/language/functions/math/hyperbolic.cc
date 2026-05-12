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
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {

absl::StatusOr<Variable> Sinh(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::sinh(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::sinh(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return TransformVector(std::sinh, std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return TransformVector(std::sinh, std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return TransformVector(std::sinh, std::get<float4>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<Variable> Asinh(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::asinh(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::asinh(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return TransformVector(std::asinh, std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return TransformVector(std::asinh, std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return TransformVector(std::asinh, std::get<float4>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<Variable> Cosh(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::cosh(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::cosh(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return TransformVector(std::cosh, std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return TransformVector(std::cosh, std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return TransformVector(std::cosh, std::get<float4>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<Variable> Acosh(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::acosh(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::acosh(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return TransformVector(std::acosh, std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return TransformVector(std::acosh, std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return TransformVector(std::acosh, std::get<float4>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<Variable> Tanh(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::tanh(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::tanh(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return TransformVector(std::tanh, std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return TransformVector(std::tanh, std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return TransformVector(std::tanh, std::get<float4>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

absl::StatusOr<Variable> Atanh(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_FloatValue:
      return std::atanh(std::get<float>(value));
    case Literal::kValue_DoubleValue:
      return std::atanh(std::get<double>(value));
    case Literal::kValue_Float2Value:
      return TransformVector(std::atanh, std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return TransformVector(std::atanh, std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return TransformVector(std::atanh, std::get<float4>(value));
    default:
      return absl::InvalidArgumentError(
          "input must be a floating-point type or a floatN type.");
  }
}

}  // namespace

void RegisterMathHyperbolicFunctions(BaseRecipeSystem* recipe_system) {
  recipe_system->RegisterFunction(
      "Sinh",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Sinh(value);
      });
  recipe_system->RegisterFunction(
      "Asinh",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Asinh(value);
      });
  recipe_system->RegisterFunction(
      "Cosh",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Cosh(value);
      });
  recipe_system->RegisterFunction(
      "Acosh",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Acosh(value);
      });
  recipe_system->RegisterFunction(
      "Tanh",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Tanh(value);
      });
  recipe_system->RegisterFunction(
      "Atanh",
      [](const recipe::Variable& value) -> absl::StatusOr<recipe::Variable> {
        return Atanh(value);
      });
}

}  // namespace imp::recipe
