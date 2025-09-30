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

#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/math/mat.h"
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {

inline const std::vector<std::string> kIds = {"0",  "1",  "2",  "3", "4",  "5",
                                              "6",  "7",  "8",  "9", "10", "11",
                                              "12", "13", "14", "15"};

recipe::Variables Extract2x2(const mat2f& input) {
  recipe::Variables variables;
  // a matrix in filament is in column-major order, so i is the column and j is
  // the row.
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      variables[kIds[i * 2 + j]] = input[i][j];
    }
  }
  return variables;
}

recipe::Variables Extract3x3(const mat3f& input) {
  recipe::Variables variables;
  // a matrix in filament is in column-major order, so i is the column and j is
  // the row.
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      variables[kIds[i * 3 + j]] = input[i][j];
    }
  }
  return variables;
}

recipe::Variables Extract4x4(const mat4f& input) {
  recipe::Variables variables;
  // a matrix in filament is in column-major order, so i is the column and j is
  // the row.
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      variables[kIds[i * 4 + j]] = input[i][j];
    }
  }
  return variables;
}

absl::StatusOr<float> Determinant(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_Mat2fValue:
      return det(std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return det(std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return det(std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError("input must be a matNf type.");
  }
}

absl::StatusOr<recipe::Variable> InvertMatrix(const recipe::Variable& matrix) {
  switch (matrix.index()) {
    case Literal::kValue_Mat2fValue:
      return inverse(std::get<mat2f>(matrix));
    case Literal::kValue_Mat3fValue:
      return inverse(std::get<mat3f>(matrix));
    case Literal::kValue_Mat4fValue:
      return inverse(std::get<mat4f>(matrix));
    default:
      return absl::InvalidArgumentError("input must be a matNf type.");
  }
}

absl::StatusOr<recipe::Variable> Transpose(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_Mat2fValue:
      return transpose(std::get<mat2f>(value));
    case Literal::kValue_Mat3fValue:
      return transpose(std::get<mat3f>(value));
    case Literal::kValue_Mat4fValue:
      return transpose(std::get<mat4f>(value));
    default:
      return absl::InvalidArgumentError("input must be a matNf type.");
  }
}

}  // namespace

void RegisterMathMatrixFunctions(BaseRecipeSystem* recipe_system) {
  recipe_system->RegisterFunction(
      "Combine2x2",
      [](float a, float b, float c, float d) { return mat2f(a, b, c, d); });

  recipe_system->RegisterFunction(
      "Combine3x3",
      [](float a, float b, float c, float d, float e, float f, float g, float h,
         float i) { return mat3f(a, b, c, d, e, f, g, h, i); });

  recipe_system->RegisterFunction(
      "Combine4x4", [](float a, float b, float c, float d, float e, float f,
                       float g, float h, float i, float j, float k, float l,
                       float m, float n, float o, float p) {
        return mat4f(a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p);
      });

  recipe_system->RegisterFunction(
      "Extract2x2", [](mat2f input) { return Extract2x2(input); });
  recipe_system->RegisterFunction(
      "Extract3x3", [](mat3f input) { return Extract3x3(input); });
  recipe_system->RegisterFunction(
      "Extract4x4", [](mat4f input) { return Extract4x4(input); });

  recipe_system->RegisterFunction(
      "Determinant", [](recipe::Variable value) -> absl::StatusOr<float> {
        return Determinant(value);
      });

  recipe_system->RegisterFunction(
      "InvertMatrix",
      [](recipe::Variable matrix) -> absl::StatusOr<recipe::Variable> {
        return InvertMatrix(matrix);
      });

  recipe_system->RegisterFunction(
      "Transpose",
      [](recipe::Variable value) -> absl::StatusOr<recipe::Variable> {
        return Transpose(value);
      });
}

}  // namespace imp::recipe
