// Copyright 2025 Google LLC
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

#include "core/recipes/language/unary_expression.h"

#include <variant>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/variant.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_traits.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {

namespace {

struct EvaluateUnaryExpressionVisitor {
  template <typename InputT>
  Variable operator()(const InputT& input) {
    switch (op) {
      case UnaryExpression::ABSOLUTE:
        if constexpr (recipe_traits::kIsAbsAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return abs(input);
          }
        }
        break;
      case UnaryExpression::SQRT:
        if constexpr (recipe_traits::kIsSqrtAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return sqrt(input);
          }
        }
        break;
      case UnaryExpression::LOG:
        if constexpr (recipe_traits::kIsLogAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return log(input);
          }
        }
        break;
      case UnaryExpression::SIN:
        if constexpr (recipe_traits::kIsSinAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Sin(input);
          }
        }
        break;
      case UnaryExpression::COS:
        if constexpr (recipe_traits::kIsCosAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Cos(input);
          }
        }
        break;
      case UnaryExpression::TAN:
        if constexpr (recipe_traits::kIsTanAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Tan(input);
          }
        }
        break;
      case UnaryExpression::ASIN:
        if constexpr (recipe_traits::kIsAsinAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Asin(input);
          }
        }
        break;
      case UnaryExpression::ACOS:
        if constexpr (recipe_traits::kIsAcosAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Acos(input);
          }
        }
        break;
      case UnaryExpression::ATAN:
        if constexpr (recipe_traits::kIsAtanAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Atan(input);
          }
        }
        break;
      case UnaryExpression::SIGN:
        if constexpr (recipe_traits::kIsSignAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Sign(input);
          }
        }
        break;
      case UnaryExpression::NORMALIZE:
        if constexpr (recipe_traits::kIsNormalizeAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return normalize(input);
          }
        }
        break;
      case UnaryExpression::NOT:
        if constexpr (recipe_traits::kIsNotAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return !input;
          }
        } else if constexpr (recipe_traits::kIsBitwiseNotAvailable<InputT>) {
          using ResultT = decltype(input);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return ~input;
          }
        }
        break;
      default:
        break;
    }

    return std::monostate();
  }

  UnaryExpression::UnaryOps op;
};

}  // namespace

absl::StatusOr<Variable> EvaluateUnaryExpression(
    const UnaryExpression::UnaryOps& op, const Variable& input) {
  Variable result = std::visit(EvaluateUnaryExpressionVisitor{.op = op}, input);

  if (absl::holds_alternative<std::monostate>(result)) {
    return absl::InternalError(absl::StrFormat(
        "invalid unary_expression %s %s", ToTypeName(input),
        proto::EnumMetaData<UnaryExpression::UnaryOps>::GetName(op)));
  }

  return result;
}

}  // namespace imp::recipe
