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

#include "core/recipes/language/recipe_runtime_graph.h"

#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/common/registry.h"
#include "core/common/type_traits.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_runtime_event.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_system.h"
#include "core/recipes/language/recipe_traits.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/view/base_view.h"
#include "robin_map/include/tsl/robin_map.h"
#include "mediapipe/framework/port/status_macros.h"

// TODO Add tests to test caching, e.g. test that values are cached
// correctly and that any nodes that mutate state clears the cache.
namespace imp {

using Variable = recipe::Variable;
using Variables = recipe::Variables;
using Args = recipe::Args;

using recipe::kDefaultOutputSocketName;

namespace {

std::string GetSocketVariableName(const NodeId& node_id,
                                  absl::string_view socket_name) {
  return absl::StrFormat("%s_%s_SOCKET_VALUE", recipe::NodeIdToString(node_id),
                         socket_name);
}

absl::Status CacheSocketValue(RecipeScope& scope, const NodeId& node_id,
                              absl::string_view socket_name,
                              const Variable& value) {
  std::string socket_variable_name =
      GetSocketVariableName(node_id, socket_name);

  std::optional<std::reference_wrapper<Variable>> variable =
      scope.GetVariable(socket_variable_name);

  if (!variable) {
    // Variable not found. Need to declare it in the scope.
    VariableDeclaration declaration{
        .name = socket_variable_name,
        .type = VariableDeclaration::Type(value.index()),
        .init_value = Literal{.value = value},
    };

    return scope.DeclareVariable(declaration);
  } else {
    // Variable is already declared. Updating its value.
    (*variable).get() = value;
    return absl::OkStatus();
  }
}

std::optional<Variable> RetrieveCachedSocketValue(
    RecipeScope& scope, const NodeId& node_id, absl::string_view socket_name) {
  return scope.GetVariable(GetSocketVariableName(node_id, socket_name));
}

}  // namespace

absl::StatusOr<std::unique_ptr<RecipeRuntimeGraph>>
RecipeRuntimeGraph::CreateRuntimeGraph(BaseView& view,
                                       const RecipeGraph& graph) {
  auto runtime_graph = absl::WrapUnique(new RecipeRuntimeGraph(view, graph));

  for (const RecipeNode& node : graph.recipe_nodes) {
    auto [_, inserted] = runtime_graph->node_map_.insert({node.id, &node});
    if (!inserted) {
      return absl::FailedPreconditionError(absl::StrFormat(
          "Found duplicate RecipeNode at NodeId %u", node.id.index));
    }

    const EventNode* event_node = node.event_node();
    if (event_node) {
      auto [_, inserted] = runtime_graph->event_node_map_.insert(
          {event_node->event_name, &node});
      if (!inserted) {
        return absl::FailedPreconditionError(
            absl::StrFormat("Found EventNode with duplicate event name %s",
                            event_node->event_name));
      }
    }
  }

  return runtime_graph;
}

RecipeRuntimeGraph::RecipeRuntimeGraph(BaseView& view, const RecipeGraph& graph)
    : recipe_system_(view.GetRegistry().GetOrCreate<RecipeSystem>(view)),
      graph_(graph) {}
namespace {
// Execute an assignment operation on a given variable to a target variable in
// the given scope.
absl::Status ExecuteAssignmentOperation(
    absl::string_view target_name, Variable& target, const Variable& value,
    const AssignmentStatement::AssignmentOps& op, RecipeScope& scope) {
#ifdef IMP_ENABLE_RECIPE_EXPERIMENTAL
  // If the target type is unknown, overwrite the variable with the new type.
  if (recipe::ToType(target) ==
      VariableDeclaration::Type::UNKNOWN_VARIABLE_TYPE) {
    VariableDeclaration declaration{
        .name = std::string(target_name),
        .type = recipe::ToType(value),
        .init_value = Literal{.value = value},
    };
    return scope.OverwriteVariable(declaration);
  }
#endif

  return std::visit(
      [&op](auto&& left, auto&& right) -> absl::Status {
        using LeftT = std::decay_t<decltype(left)>;
        using RightT = std::decay_t<decltype(right)>;

        switch (op) {
          case AssignmentStatement::AssignmentOps::ASSIGN:
            if constexpr (recipe_traits::kIsAssignAvailable<LeftT, RightT>) {
              left = right;
            }
            break;
          case AssignmentStatement::AssignmentOps::ADD_ASSIGN:
            if constexpr (recipe_traits::kIsAddAssignAvailable<LeftT, RightT>) {
              left += right;
            }
            break;
          case AssignmentStatement::AssignmentOps::SUBTRACT_ASSIGN:
            if constexpr (recipe_traits::kIsSubtractAssignAvailable<LeftT,
                                                                    RightT>) {
              left -= right;
            }
            break;
          case AssignmentStatement::AssignmentOps::MULTIPLY_ASSIGN:
            if constexpr (recipe_traits::kIsMultiplyAssignAvailable<LeftT,
                                                                    RightT>) {
              left *= right;
            }
            break;
          case AssignmentStatement::AssignmentOps::DIVIDE_ASSIGN:
            if constexpr (recipe_traits::kIsDivideAssignAvailable<LeftT,
                                                                  RightT>) {
              left /= right;
            }
            break;
          default:
            return absl::InternalError(absl::StrFormat(
                "Failed to execute assignment_expression. Left Type: %s "
                "Operator: %s Right Type: %s",
                type_traits::kTypeName<LeftT>,
                proto::EnumMetaData<
                    AssignmentStatement::AssignmentOps>::GetName(op),
                type_traits::kTypeName<RightT>));
        }

        return absl::OkStatus();
      },
      target, value);
}

}  // namespace

absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::ExecuteAssignmentStatement(
    const AssignmentStatement& statement, RecipeScope& scope) const {
  // Creating a temporary scope for caching all Variables evaluated only within
  // the scope of this executable node.
  RecipeScope executable_node_scope = RecipeScope{&scope};
  MP_ASSIGN_OR_RETURN(
      Variable value_result,
      EvaluateValueConnection(statement.value, executable_node_scope));

  if (statement.targets.empty()) {
    return absl::InvalidArgumentError(
        "AssignmentStatement must have at least one target.");
  } else if (statement.targets.size() == 1) {
    // Single target assignment
    absl::string_view target_name = statement.targets[0].name;
    std::optional<std::reference_wrapper<Variable>> variable =
        scope.GetVariable(target_name);
    if (!variable) {
      return absl::NotFoundError(absl::StrFormat(
          "Can't find variable named %s to assign to.", target_name));
    }
    MP_RETURN_IF_ERROR(ExecuteAssignmentOperation(target_name, *variable,
                                               value_result, statement.op,
                                               executable_node_scope));
  } else if (statement.targets.size() != 1) {
    // Multiple target assignment (unpacking)
    if (!std::holds_alternative<LiteralTuple>(value_result)) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "AssignmentStatement with multiple targets requires a LiteralTuple "
          "on the right side. Instead received: %s",
          recipe::ToTypeName(value_result)));
    }

    const LiteralTuple& tuple = std::get<LiteralTuple>(value_result);
    if (tuple.values.size() != statement.targets.size()) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "AssignmentStatement with %zu targets requires the right side "
          "LiteralTuple to have %zu elements, but got %zu.",
          statement.targets.size(), statement.targets.size(),
          tuple.values.size()));
    }

    for (size_t i = 0; i < statement.targets.size(); ++i) {
      absl::string_view target_name = statement.targets[i].name;
      std::optional<std::reference_wrapper<Variable>> variable =
          scope.GetVariable(target_name);
      if (!variable) {
        return absl::NotFoundError(absl::StrFormat(
            "Can't find variable named %s to assign to.", target_name));
      }
      MP_RETURN_IF_ERROR(ExecuteAssignmentOperation(
          target_name, *variable, tuple.values[i].value, statement.op, scope));
    }
  }

  return ExecuteNode(statement.next_node, scope);
}

absl::StatusOr<Variable> RecipeRuntimeGraph::EvaluateUnaryExpression(
    const UnaryExpression& expression, RecipeScope& scope) const {
  MP_ASSIGN_OR_RETURN(Variable previous_result,
                   EvaluateValueConnection(expression.input, scope));

  if (absl::holds_alternative<absl::monostate>(previous_result)) {
    return absl::InvalidArgumentError(
        "Cannot compute unary expression with empty input.");
  }

  return std::visit(
      [&expression](auto&& input) -> absl::StatusOr<Variable> {
        using InputT = std::decay_t<decltype(input)>;

        std::string op_name;
        Variable result;

        switch (expression.op) {
          case UnaryExpression::ABSOLUTE:
            op_name = "abs";
            if constexpr (recipe_traits::kIsAbsAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = abs(input);
              }
            }
            break;
          case UnaryExpression::SQRT:
            op_name = "sqrt";
            if constexpr (recipe_traits::kIsSqrtAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = sqrt(input);
              }
            }
            break;
          case UnaryExpression::LOG:
            op_name = "log";
            if constexpr (recipe_traits::kIsLogAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = log(input);
              }
            }
            break;
          case UnaryExpression::SIN:
            op_name = "sin";
            if constexpr (recipe_traits::kIsSinAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = sin(input);
              }
            }
            break;
          case UnaryExpression::COS:
            op_name = "cos";
            if constexpr (recipe_traits::kIsCosAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = cos(input);
              }
            }
            break;
          case UnaryExpression::TAN:
            op_name = "tan";
            if constexpr (recipe_traits::kIsTanAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = tan(input);
              }
            }
            break;
          case UnaryExpression::ASIN:
            op_name = "asin";
            if constexpr (recipe_traits::kIsAsinAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = asin(input);
              }
            }
            break;
          case UnaryExpression::ACOS:
            op_name = "acos";
            if constexpr (recipe_traits::kIsAcosAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = acos(input);
              }
            }
            break;
          case UnaryExpression::ATAN:
            op_name = "atan";
            if constexpr (recipe_traits::kIsAtanAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = atan(input);
              }
            }
            break;
          case UnaryExpression::SIGN:
            op_name = "sign";
            if constexpr (recipe_traits::kIsSignAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = recipe::Sign(input);
              }
            }
            break;
          case UnaryExpression::NORMALIZE:
            op_name = "normalize";
            if constexpr (recipe_traits::kIsNormalizeAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = normalize(input);
              }
            }
            break;
          case UnaryExpression::NOT:
            op_name = "not";
            if constexpr (recipe_traits::kIsNotAvailable<InputT>) {
              using ResultT = decltype(input);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = !input;
              }
            }
            break;
          default:
            return absl::NotFoundError("Unknown unary expression operator.");
            break;
        }

        if (!absl::holds_alternative<absl::monostate>(result)) {
          imp::output::Recipe("executed unary_expression %s %s",
                              type_traits::kTypeName<InputT>, op_name);
        } else {
          return absl::InternalError(
              absl::StrFormat("invalid unary_expression %s %s",
                              type_traits::kTypeName<InputT>, op_name));
        }

        return result;
      },
      previous_result);
}

absl::StatusOr<Variable> RecipeRuntimeGraph::EvaluateBinaryExpression(
    const BinaryExpression& expression, RecipeScope& scope) const {
  MP_ASSIGN_OR_RETURN(Variable left_result,
                   EvaluateValueConnection(expression.left, scope));

  MP_ASSIGN_OR_RETURN(Variable right_result,
                   EvaluateValueConnection(expression.right, scope));

#if IMP_ENABLE_RECIPE_EXPERIMENTAL
  // Support AND and OR expressions with empty left or right, treating them as
  // false.
  if (absl::holds_alternative<absl::monostate>(left_result) ||
      absl::holds_alternative<absl::monostate>(right_result)) {
    if (expression.op == BinaryExpression::AND) {
      return Variable(false);
    } else if (expression.op == BinaryExpression::OR) {
      bool result = recipe::CoerceToBool(left_result) ||
                    recipe::CoerceToBool(right_result);
      return Variable(result);
    } else {
      return absl::InvalidArgumentError(
          "Cannot compute binary expression with empty left or right.");
    }
  }
#else
  if (absl::holds_alternative<absl::monostate>(left_result) ||
      absl::holds_alternative<absl::monostate>(right_result)) {
    return absl::InvalidArgumentError(
        "Cannot compute binary expression with empty left or right.");
  }
#endif

  return std::visit(
      [&expression](auto&& left, auto&& right) -> absl::StatusOr<Variable> {
        using LeftT = std::decay_t<decltype(left)>;
        using RightT = std::decay_t<decltype(right)>;

        std::string op_name;
        Variable result;

        switch (expression.op) {
          case BinaryExpression::ADD:
            op_name = "add";
            if constexpr (recipe_traits::kIsAddAvailable<LeftT, RightT>) {
              using ResultT = decltype(left + right);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = left + right;
              }
            }
            break;
          case BinaryExpression::SUBTRACT:
            op_name = "subtract";
            if constexpr (recipe_traits::kIsSubtractAvailable<LeftT, RightT>) {
              using ResultT = decltype(left - right);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = left - right;
              }
            }
            break;
          case BinaryExpression::MULTIPLY:
            op_name = "multiply";
            if constexpr (recipe_traits::kIsMultiplyAvailable<LeftT, RightT>) {
              using ResultT = decltype(left * right);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = left * right;
              }
            }
            break;
          case BinaryExpression::DIVIDE:
            op_name = "divide";
            if constexpr (recipe_traits::kIsDivideAvailable<LeftT, RightT>) {
              using ResultT = decltype(left / right);
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = left / right;
              }
            }
            break;
          case BinaryExpression::MOD:
            op_name = "mod";
            if constexpr (recipe_traits::kIsModAvailable<LeftT, RightT>) {
              if constexpr (std::is_same_v<LeftT, int> &&
                            std::is_same_v<RightT, int>) {
                result = left % right;
              } else {
                result = std::fmod(left, right);
              }
            }
            break;
          case BinaryExpression::EQUALS:
            op_name = "equals";
            if constexpr (recipe_traits::kIsEqualsAvailable<LeftT, RightT>) {
              result = left == right;
            }
            break;
          case BinaryExpression::NOT_EQUALS:
            op_name = "not_equals";
            if constexpr (recipe_traits::kIsNotEqualsAvailable<LeftT, RightT>) {
              result = left != right;
            }
            break;
          case BinaryExpression::GREATER_THAN:
            op_name = "greater_than";
            if constexpr (recipe_traits::kIsGreaterThanAvailable<LeftT,
                                                                 RightT>) {
              result = left > right;
            }
            break;
          case BinaryExpression::LESS_THAN:
            op_name = "less_than";
            if constexpr (recipe_traits::kIsLessThanAvailable<LeftT, RightT>) {
              result = left < right;
            }
            break;
          case BinaryExpression::GREATER_THAN_OR_EQUAL:
            op_name = "greater_than_or_equal";
            if constexpr (recipe_traits::kIsGreaterThanOrEqualAvailable<
                              LeftT, RightT>) {
              result = left >= right;
            }
            break;
          case BinaryExpression::LESS_THAN_OR_EQUAL:
            op_name = "less_than_or_equal";
            if constexpr (recipe_traits::kIsLessThanOrEqualAvailable<LeftT,
                                                                     RightT>) {
              result = left <= right;
            }
            break;
          case BinaryExpression::AND:
            op_name = "and";
            if constexpr (recipe_traits::kIsAndAvailable<LeftT, RightT>) {
              result = left && right;
            }
            break;
          case BinaryExpression::OR:
            op_name = "or";
            if constexpr (recipe_traits::kIsOrAvailable<LeftT, RightT>) {
              result = left || right;
            }
            break;
          case BinaryExpression::DOT:
            op_name = "dot";
            if constexpr (recipe_traits::kIsDotAvailable<LeftT, RightT>) {
              using ResultT = decltype(dot(left, right));
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = dot(left, right);
              }
            }
            break;
          case BinaryExpression::CROSS:
            op_name = "cross";
            if constexpr (recipe_traits::kIsCrossAvailable<LeftT, RightT>) {
              using ResultT = decltype(cross(left, right));
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = cross(left, right);
              }
            }
            break;
          case BinaryExpression::MIN:
            op_name = "min";
            if constexpr (std::is_same_v<LeftT, RightT> &&
                          recipe_traits::kIsLessThanAvailable<LeftT, RightT>) {
              using ResultT = decltype(std::min(left, right));
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = std::min(left, right);
              }
            }
            break;
          case BinaryExpression::MAX:
            op_name = "max";
            if constexpr (std::is_same_v<LeftT, RightT> &&
                          recipe_traits::kIsGreaterThanAvailable<LeftT,
                                                                 RightT>) {
              using ResultT = decltype(std::max(left, right));
              if constexpr (std::is_constructible_v<Variable, ResultT>) {
                result = std::max(left, right);
              }
            }
            break;
          default:
            return absl::NotFoundError("Unknown binary expression operator.");
            break;
        }

        if (!absl::holds_alternative<absl::monostate>(result)) {
          imp::output::Recipe("executed binary_expression %s %s %s",
                              type_traits::kTypeName<LeftT>, op_name,
                              type_traits::kTypeName<RightT>);
        } else {
          return absl::InternalError(
              absl::StrFormat("invalid binary_expression %s %s %s",
                              type_traits::kTypeName<LeftT>, op_name,
                              type_traits::kTypeName<RightT>));
        }

        return result;
      },
      left_result, right_result);
}

absl::StatusOr<recipe::ReturnValue> RecipeRuntimeGraph::EvaluateCallExpression(
    const CallExpression& call_expression, RecipeScope& scope) const {
  Args evaluated_args;
  evaluated_args.reserve(call_expression.args.size());
  for (const ValueConnection& arg_value_connection : call_expression.args) {
    MP_ASSIGN_OR_RETURN(Variable evaluated_arg,
                     EvaluateValueConnection(arg_value_connection, scope));

    // TODO: Combine experimental and non-experimental code paths.
#if IMP_ENABLE_RECIPE_EXPERIMENTAL
    // Evaluating an std::monostate will coerce to the default value for the
    // type in the registered functions.
    evaluated_args.push_back(evaluated_arg);
#else
    if (absl::holds_alternative<absl::monostate>(evaluated_arg)) {
      return absl::InternalError("Token for function arg must return a value.");

    } else {
      evaluated_args.push_back(evaluated_arg);
    }
#endif
  }

  recipe::NamedArgs evaluated_named_args;
  evaluated_named_args.reserve(call_expression.named_args.size());
  for (const auto& [arg_name, arg_value_connection] :
       call_expression.named_args) {
    MP_ASSIGN_OR_RETURN(Variable evaluated_arg,
                     EvaluateValueConnection(arg_value_connection, scope));
    evaluated_named_args.push_back(std::make_pair(arg_name, evaluated_arg));
  }

  return recipe_system_.ExecuteFunction(call_expression.name, evaluated_args,
                                        evaluated_named_args);
}

absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::ExecuteCallStatement(const NodeId& node_id,
                                         const CallStatement& call_statement,
                                         RecipeScope& scope) const {
  RecipeScope executable_node_scope = RecipeScope{&scope};
  MP_ASSIGN_OR_RETURN(
      recipe::ReturnValue result,
      EvaluateCallExpression(call_statement.expression, executable_node_scope));

  if (result.async_values) {
    if (!result.async_values->Ready()) {
      return absl::InternalError(absl::StrFormat(
          "Cannot call async function %s that returns unready variables with "
          "CallStatement. Use AsyncCallStatement "
          "instead.",
          call_statement.expression.name));
    } else {
      MP_ASSIGN_OR_RETURN(Variables async_values, result.async_values->Get());
      MP_RETURN_IF_ERROR(CacheSocketValues(scope, node_id, async_values));
    }
  }

  MP_RETURN_IF_ERROR(CacheSocketValues(scope, node_id, result.values));

  return ExecuteNode(call_statement.next_node, scope);
}

absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::ExecuteAsyncCallStatement(
    const NodeId& node_id, const AsyncCallStatement& call_statement,
    RecipeScope& scope) const {
  RecipeScope executable_node_scope = RecipeScope{&scope};
  MP_ASSIGN_OR_RETURN(
      recipe::ReturnValue result,
      EvaluateCallExpression(call_statement.expression, executable_node_scope));

  ExecutionResult execution_result;

  if (!result.values.empty()) {
    // Cache sync return values.
    MP_RETURN_IF_ERROR(CacheSocketValues(scope, node_id, result.values));
  }

  Future<absl::Status> cache_return_values_future(absl::OkStatus());
  if (result.async_values) {
    cache_return_values_future = result.async_values->Then(
        [&scope, node_id, this](Variables return_values) -> absl::Status {
          MP_RETURN_IF_ERROR(CacheSocketValues(scope, node_id, return_values));
          return absl::OkStatus();
        });
  }

  AsyncExecutionHandle async_handle = cache_return_values_future.Then(
      [on_done_node_id =
           call_statement.on_done_node.node_id]() -> std::optional<NodeId> {
        return on_done_node_id;
      });

  execution_result.async_execution_handles.push_back(async_handle);

  MP_ASSIGN_OR_RETURN(ExecutionResult next_result,
                   ExecuteNode(call_statement.next_node, scope));
  execution_result.Combine(next_result);
  return execution_result;
}

absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::ExecuteBranchStatement(const BranchStatement& statement,
                                           RecipeScope& scope) const {
  // Creating a temporary scope for caching all Variables evaluated only
  // within the scope of this executable node.
  RecipeScope executable_node_scope = RecipeScope{&scope};
  MP_ASSIGN_OR_RETURN(
      Variable test_result,
      EvaluateValueConnection(statement.test, executable_node_scope));

#if IMP_ENABLE_RECIPE_EXPERIMENTAL
  // TODO: Support non-bool test results without coercion in the
  // graph. Attempt to coerce the test result to a bool.
  std::optional<bool> coerced_test_result = recipe::CoerceToBool(test_result);
  if (!coerced_test_result.has_value()) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Test must return a bool. Instead received %s",
                        recipe::ToTypeName(test_result)));
  }
  if (*coerced_test_result) {
    return ExecuteNode(statement.true_next_node, scope);
  } else {
    return ExecuteNode(statement.false_next_node, scope);
  }
#else
  if (!absl::holds_alternative<bool>(test_result)) {
    return absl::InvalidArgumentError("test must return a bool.");
  }
  if (absl::get<bool>(test_result)) {
    return ExecuteNode(statement.true_next_node, scope);
  } else {
    return ExecuteNode(statement.false_next_node, scope);
  }
#endif
}

absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::ExecuteVariableDeclarationStatement(
    const VariableDeclarationStatement& statement, RecipeScope& scope) const {
  MP_RETURN_IF_ERROR(scope.DeclareVariable(statement.declaration));
  return ExecuteNode(statement.next_node, scope);
}

absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::ExecuteLoopStatement(const NodeId& node_id,
                                         const LoopStatement& statement,
                                         RecipeScope& scope) const {
  // Creating a temporary scope for caching all Variables evaluated only
  // within the scope of this executable node.
  RecipeScope executable_node_scope = RecipeScope{&scope};
  MP_ASSIGN_OR_RETURN(
      Variable increment,
      EvaluateValueConnection(statement.increment, executable_node_scope));
  MP_ASSIGN_OR_RETURN(
      Variable start_index,
      EvaluateValueConnection(statement.start_index, executable_node_scope));
  MP_ASSIGN_OR_RETURN(
      Variable end_index,
      EvaluateValueConnection(statement.end_index, executable_node_scope));

  if (!absl::holds_alternative<int>(increment)) {
    return absl::InvalidArgumentError("Loop increment must be an integer.");
  }

  if (!absl::holds_alternative<int>(start_index)) {
    return absl::InvalidArgumentError("Loop start_index must be an integer.");
  }

  if (!absl::holds_alternative<int>(end_index)) {
    return absl::InvalidArgumentError("Loop end index must be an integer.");
  }

  ExecutionResult result;

  int inc = absl::get<int>(increment);
  int start = absl::get<int>(start_index);
  int end = absl::get<int>(end_index);
  for (int i = start; i < end; i += inc) {
    // Set the "index" socket to reflect the current loop iteration.
    MP_RETURN_IF_ERROR(CacheSocketValue(scope, node_id, "index", Variable(i)));

    MP_ASSIGN_OR_RETURN(ExecutionResult sub_result,
                     ExecuteNode(statement.looped_node, scope));
    result.Combine(sub_result);
  }

  MP_ASSIGN_OR_RETURN(ExecutionResult next_result,
                   ExecuteNode(statement.completed_node, scope));
  result.Combine(next_result);

  return result;
}

absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::ExecuteWhileStatement(const NodeId& node_id,
                                          const WhileStatement& statement,
                                          RecipeScope& scope) const {
  // Creating a temporary scope for caching all Variables evaluated only
  // within the scope of this executable node.
  RecipeScope executable_node_scope = RecipeScope{&scope};

  MP_ASSIGN_OR_RETURN(
      Variable condition,
      EvaluateValueConnection(statement.condition, executable_node_scope));

  ExecutionResult result;
  while (absl::holds_alternative<bool>(condition) &&
         std::get<bool>(condition)) {
    MP_ASSIGN_OR_RETURN(ExecutionResult sub_result,
                     ExecuteNode(statement.looped_node, scope));

    result.Combine(sub_result);
    executable_node_scope.ClearLocalVariables();

    MP_ASSIGN_OR_RETURN(condition, EvaluateValueConnection(statement.condition,
                                                        executable_node_scope));
  }

  MP_ASSIGN_OR_RETURN(ExecutionResult next_result,
                   ExecuteNode(statement.completed_node, scope));
  result.Combine(next_result);

  return result;
}

absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::ExecuteSequenceStatement(const SequenceStatement& statement,
                                             RecipeScope& scope) const {
  ExecutionResult result;

  for (const ExecutableNodeConnection& node : statement.next_nodes) {
    MP_ASSIGN_OR_RETURN(ExecutionResult sub_result, ExecuteNode(node, scope));
    result.Combine(sub_result);
  }

  return result;
}

absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::ExecuteEventTriggerStatement(const EventTrigger& statement,
                                                 RecipeScope& scope) const {
  Variables event_args;
  for (const auto& [arg_name, arg] : statement.args) {
    // Creating a temporary scope for caching all Variables evaluated only
    // within the scope of this evaluation.
    RecipeScope executable_node_scope = RecipeScope{&scope};
    MP_ASSIGN_OR_RETURN(Variable evaluated_arg,
                     EvaluateValueConnection(arg, executable_node_scope));
#if IMP_ENABLE_RECIPE_EXPERIMENTAL
    event_args[arg_name] = evaluated_arg;
#else
    if (absl::holds_alternative<absl::monostate>(evaluated_arg)) {
      return absl::InternalError("Token for function arg must return a value.");
    } else {
      event_args[arg_name] = evaluated_arg;
    }
#endif
  }

  if (!runtime_event_listener_) {
    return absl::InternalError("RuntimeEventListener is null.");
  }

  runtime_event_listener_(RecipeRuntimeEvent{
      .name = statement.event_name, .arguments = std::move(event_args)});

  return ExecuteNode(statement.next_node, scope);
}

absl::StatusOr<Variables> RecipeRuntimeGraph::EvaluateNode(
    const NodeId& node_id, RecipeScope& scope) const {
  MP_ASSIGN_OR_RETURN(const ValueNode* value_node, GetNode<ValueNode>(node_id));

  // TODO Guard this behind IMP_RUNTIME(DEV) instead
  if constexpr (output::kEnableRecipeLog) {
    auto itr = node_map_.find(node_id);
    if (itr != node_map_.end()) {
      imp::output::Recipe("Evaluating Node %s with recipe node id %d",
                          itr->second->name, node_id.index);
    }
  }

  return std::visit(
      [this, &scope](auto&& arg) -> absl::StatusOr<Variables> {
        using T = std::decay_t<decltype(arg)>;
        Variables result;
        if constexpr (std::is_same_v<T, Identifier>) {
          imp::output::Recipe("Identifier token %s.", arg.name);
          std::optional<std::reference_wrapper<Variable>> variable =
              scope.GetVariable(arg.name);
          if (!variable) {
            return absl::FailedPreconditionError(
                absl::StrFormat("Unable to find variable %s.", arg.name));
          }
          result[std::string(kDefaultOutputSocketName)] = *variable;
          return result;
        } else if constexpr (std::is_same_v<T, UnaryExpression>) {
          imp::output::Recipe("UnaryExpression token.");
          MP_ASSIGN_OR_RETURN(Variable return_value,
                           EvaluateUnaryExpression(arg, scope));
          result[std::string(kDefaultOutputSocketName)] = return_value;
          return result;
        } else if constexpr (std::is_same_v<T, BinaryExpression>) {
          imp::output::Recipe("BinaryExpression token.");
          MP_ASSIGN_OR_RETURN(Variable return_value,
                           EvaluateBinaryExpression(arg, scope));
          result[std::string(kDefaultOutputSocketName)] = return_value;
          return result;
        } else if constexpr (std::is_same_v<T, CallExpression>) {
          imp::output::Recipe("CallExpression token.");
          MP_ASSIGN_OR_RETURN(recipe::ReturnValue result,
                           EvaluateCallExpression(arg, scope));
          if (result.async_values) {
            return absl::UnimplementedError(absl::StrFormat(
                "CallExpression ValueNode with async function %s is not "
                "supported.",
                arg.name));
          }
          return result.values;
        } else {
          return absl::NotFoundError("Unknown ValueNode type.");
        }
        return result;
      },
      value_node->value);
}

absl::StatusOr<Variable> RecipeRuntimeGraph::EvaluateValueConnection(
    const ValueConnection& value_connection, RecipeScope& scope) const {
  if (value_connection.literal_value()) {
    return value_connection.literal_value()->value;
  } else if (value_connection.socket_connection()) {
    return EvaluateSocketConnection(*value_connection.socket_connection(),
                                    scope);
  }
  return absl::InvalidArgumentError("Unknown ValueConnection type.");
}

absl::StatusOr<Variable> RecipeRuntimeGraph::EvaluateSocketConnection(
    const SocketConnection& socket_connection, RecipeScope& scope) const {
  std::optional<Variable> cached_value = RetrieveCachedSocketValue(
      scope, socket_connection.node_id, socket_connection.socket_name);
  if (cached_value) {
    return *cached_value;
  } else if (!GetNode<ValueNode>(socket_connection.node_id).ok()) {
    return absl::InternalError(
        absl::StrFormat("Referencing an uncached socket %s on node %s",
                        socket_connection.socket_name,
                        recipe::NodeIdToString(socket_connection.node_id)));
  }

  MP_ASSIGN_OR_RETURN(Variables result,
                   EvaluateNode(socket_connection.node_id, scope));

  MP_RETURN_IF_ERROR(CacheSocketValues(scope, socket_connection.node_id, result));

  auto it = result.find(std::string(socket_connection.socket_name));
  if (it == result.end()) {
    return absl::NotFoundError(absl::StrFormat("Referencing missing socket %s.",
                                               socket_connection.socket_name));
  }
  return it->second;
}

// TODO ExecutionNode currently recurses down the entire graph to
// execute the recipe graph. This does not scale well for larger graphs and we
// risk overflowing the call stack. Instead, ExecuteNode should, over a long
// execution flow, have an O(1) sized call stack.
absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::ExecuteNode(
    const ExecutableNodeConnection& executable_node_connection,
    RecipeScope& scope) const {
  if (!executable_node_connection.node_id) {
    return ExecutionResult{};
  }

  const NodeId& node_id = *executable_node_connection.node_id;

  MP_ASSIGN_OR_RETURN(const ExecutableNode* executable_node,
                   GetNode<ExecutableNode>(node_id));

  // TODO Guard this behind IMP_RUNTIME(DEV) instead
  if constexpr (output::kEnableRecipeLog) {
    auto itr = node_map_.find(node_id);
    if (itr != node_map_.end()) {
      imp::output::Recipe("Executing Node %s with recipe node id %d",
                          itr->second->name, node_id.index);
    }
  }

  return std::visit(
      [this, &scope, &node_id](auto&& arg) -> absl::StatusOr<ExecutionResult> {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, CallStatement>) {
          imp::output::Recipe("CallStatement.");
          return ExecuteCallStatement(node_id, arg, scope);
        } else if constexpr (std::is_same_v<T, AssignmentStatement>) {
          imp::output::Recipe("AssignmentStatement.");
          return ExecuteAssignmentStatement(arg, scope);
        } else if constexpr (std::is_same_v<T, BranchStatement>) {
          imp::output::Recipe("BranchStatement.");
          return ExecuteBranchStatement(arg, scope);
        } else if constexpr (std::is_same_v<T, VariableDeclarationStatement>) {
          imp::output::Recipe("VariableDeclarationStatement.");
          return ExecuteVariableDeclarationStatement(arg, scope);
        } else if constexpr (std::is_same_v<T, LoopStatement>) {
          imp::output::Recipe("LoopStatement.");
          return ExecuteLoopStatement(node_id, arg, scope);
        } else if constexpr (std::is_same_v<T, SequenceStatement>) {
          imp::output::Recipe("SequenceStatement.");
          return ExecuteSequenceStatement(arg, scope);
        } else if constexpr (std::is_same_v<T, EventTrigger>) {
          imp::output::Recipe("EventTrigger.");
          return ExecuteEventTriggerStatement(arg, scope);
        } else if constexpr (std::is_same_v<T, AsyncCallStatement>) {
          imp::output::Recipe("AsyncCallStatement.");
          return ExecuteAsyncCallStatement(node_id, arg, scope);
        } else if constexpr (std::is_same_v<T, WhileStatement>) {
          imp::output::Recipe("WhileStatement.");
          return ExecuteWhileStatement(node_id, arg, scope);
        } else {
          return absl::NotFoundError("Unknown statement type.");
        }
      },
      executable_node->statement);
}

absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::TriggerEvent(const RecipeRuntimeEvent& event,
                                 RecipeScope* scope) const {
  auto it = event_node_map_.find(std::string(event.name));
  if (it == event_node_map_.end()) {
    return absl::NotFoundError(
        absl::StrFormat("RecipeEvent %s not found. Skipping.", event.name));
  }

  const RecipeNode& node = *it->second;
  const EventNode& event_node = *node.event_node();

  MP_RETURN_IF_ERROR(CacheSocketValues(*scope, node.id, event.arguments));

  absl::StatusOr<ExecutionResult> result =
      ExecuteNode(event_node.next_node, *scope);

  return result;
}

absl::StatusOr<RecipeRuntimeGraph::ExecutionResult>
RecipeRuntimeGraph::ResumeExecution(const AsyncExecutionHandle& handle,
                                    RecipeScope* scope) const {
  if (!handle.Ready()) {
    return absl::FailedPreconditionError("AsyncExecutionHandle is not ready.");
  }

  MP_ASSIGN_OR_RETURN(std::optional<NodeId> node_id, handle.Get());

  absl::StatusOr<ExecutionResult> result =
      ExecuteNode(ExecutableNodeConnection{.node_id = node_id}, *scope);

  return result;
}

absl::Status RecipeRuntimeGraph::CacheSocketValues(
    RecipeScope& scope, const NodeId& node_id,
    const Variables& socket_values) const {
  for (const auto& [socket_name, value] : socket_values) {
    MP_RETURN_IF_ERROR(CacheSocketValue(scope, node_id, socket_name, value));
  }

  if (socket_value_listener_) {
    socket_value_listener_(node_id, socket_values);
  }

  return absl::OkStatus();
}

void RecipeRuntimeGraph::SetRuntimeEventListener(
    Invocable<void(RecipeRuntimeEvent)> listener) {
  runtime_event_listener_ = std::move(listener);
}

void RecipeRuntimeGraph::SetSocketValueListener(
    Invocable<void(const NodeId& id, const recipe::Variables&)> listener) {
  socket_value_listener_ = std::move(listener);
}

}  // namespace imp
