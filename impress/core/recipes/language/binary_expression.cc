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

#include "core/recipes/language/binary_expression.h"

#include <type_traits>
#include <variant>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "core/math/almost_equal.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_traits.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {

struct EvaluateBinaryExpressionVisitor {
  template <typename LeftT, typename RightT>
  Variable operator()(LeftT left, RightT right) const {
    switch (op) {
      case BinaryExpression::ADD:
        if constexpr (recipe_traits::kIsAddAvailable<LeftT, RightT>) {
          using ResultT = decltype(left + right);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return left + right;
          }
        }
        break;
      case BinaryExpression::SUBTRACT:
        if constexpr (recipe_traits::kIsSubtractAvailable<LeftT, RightT>) {
          using ResultT = decltype(left - right);
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return left - right;
          }
        }
        break;
      case BinaryExpression::MULTIPLY:
        if constexpr (recipe_traits::kIsMultiplyAvailable<LeftT, RightT>) {
          using ResultT = decltype(recipe::Multiply(left, right));
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Multiply(left, right);
          }
        }
        break;
      case BinaryExpression::DIVIDE:
        if constexpr (recipe_traits::kIsDivideAvailable<LeftT, RightT>) {
          using ResultT = decltype(recipe::Divide(left, right));
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Divide(left, right);
          }
        }
        break;
      case BinaryExpression::MOD:
        if constexpr (recipe_traits::kIsModAvailable<LeftT, RightT>) {
          using ResultT = decltype(recipe::Fmod(left, right));
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Fmod(left, right);
          }
        }
        break;
      case BinaryExpression::EQUALS:
        if constexpr (recipe_traits::kIsEqualsAvailable<LeftT, RightT>) {
          if constexpr (std::is_same_v<LeftT, RightT>) {
            // Check if AlmostEqual supports this type.
            // We support float, vectors, quaternions, and matrices.
            // Variable can hold these types as aliases from imp namespace.
            if constexpr (std::is_floating_point_v<LeftT>) {
              return imp::AlmostEqual(left, right);
            } else if constexpr (imp::kIsAnyOf<LeftT, imp::float2, imp::float3,
                                               imp::float4, imp::quat>) {
              return imp::AlmostEqual(left, right);
            }
          }
          return left == right;
        }
        break;
      case BinaryExpression::NOT_EQUALS:
        if constexpr (recipe_traits::kIsNotEqualsAvailable<LeftT, RightT>) {
          return left != right;
        }
        break;
      case BinaryExpression::GREATER_THAN:
        if constexpr (recipe_traits::kIsGreaterThanAvailable<LeftT, RightT>) {
          return left > right;
        }
        break;
      case BinaryExpression::LESS_THAN:
        if constexpr (recipe_traits::kIsLessThanAvailable<LeftT, RightT>) {
          return left < right;
        }
        break;
      case BinaryExpression::GREATER_THAN_OR_EQUAL:
        if constexpr (recipe_traits::kIsGreaterThanOrEqualAvailable<LeftT,
                                                                    RightT>) {
          return left >= right;
        }
        break;
      case BinaryExpression::LESS_THAN_OR_EQUAL:
        if constexpr (recipe_traits::kIsLessThanOrEqualAvailable<LeftT,
                                                                 RightT>) {
          return left <= right;
        }
        break;
      case BinaryExpression::AND:
        if constexpr (recipe_traits::kIsAndAvailable<LeftT, RightT>) {
          return left && right;
        } else if constexpr (recipe_traits::kIsBitwiseAndAvailable<LeftT,
                                                                   RightT>) {
          return left & right;
        }
        break;
      case BinaryExpression::OR:
        if constexpr (recipe_traits::kIsOrAvailable<LeftT, RightT>) {
          return left || right;
        } else if constexpr (recipe_traits::kIsBitwiseOrAvailable<LeftT,
                                                                  RightT>) {
          return left | right;
        }
        break;
      case BinaryExpression::XOR:
        if constexpr (recipe_traits::kIsXorAvailable<LeftT, RightT>) {
          return left != right;
        } else if constexpr (recipe_traits::kIsBitwiseXorAvailable<LeftT,
                                                                   RightT>) {
          return left ^ right;
        }
        break;
      case BinaryExpression::DOT:
        if constexpr (recipe_traits::kIsDotAvailable<LeftT, RightT>) {
          using ResultT = decltype(dot(left, right));
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return dot(left, right);
          }
        }
        break;
      case BinaryExpression::CROSS:
        if constexpr (recipe_traits::kIsCrossAvailable<LeftT, RightT>) {
          using ResultT = decltype(cross(left, right));
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return cross(left, right);
          }
        }
        break;
      case BinaryExpression::MIN:
        if constexpr (recipe_traits::kIsMinAvailable<LeftT, RightT>) {
          using ResultT = decltype(recipe::Min(left, right));
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Min(left, right);
          }
        }
        break;
      case BinaryExpression::MAX:
        if constexpr (recipe_traits::kIsMaxAvailable<LeftT, RightT>) {
          using ResultT = decltype(recipe::Max(left, right));
          if constexpr (std::is_constructible_v<Variable, ResultT>) {
            return recipe::Max(left, right);
          }
        }
        break;
      default:
        break;
    }

    return std::monostate();
  }

  BinaryExpression::BinaryOps op;
};

absl::StatusOr<Variable> EvaluateBinaryExpression(
    const BinaryExpression::BinaryOps& op, const Variable& left,
    const Variable& right) {
  Variable result =
      std::visit(EvaluateBinaryExpressionVisitor{.op = op}, left, right);

  if (std::holds_alternative<std::monostate>(result)) {
    return absl::InternalError(absl::StrFormat(
        "invalid binary_expression %s %s %s", recipe::ToTypeName(left),
        proto::EnumMetaData<BinaryExpression::BinaryOps>::GetName(op),
        recipe::ToTypeName(right)));
  }

  return result;
}

}  // namespace imp::recipe
