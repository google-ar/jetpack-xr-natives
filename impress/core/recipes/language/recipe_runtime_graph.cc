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

#include <sys/types.h>

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/variant.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/common/registry.h"
#include "core/recipes/language/binary_expression.h"
#include "core/recipes/language/recipe_async_execution_manager.h"
#include "core/recipes/language/recipe_custom_statement.h"
#include "core/recipes/language/recipe_execution_context.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_runtime_event.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_system.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/recipes/language/unary_expression.h"
#include "core/view/base_view.h"
#include "mediapipe/framework/port/status_macros.h"

// TODO Add tests to test caching, e.g. test that values are cached
// correctly and that any nodes that mutate state clears the cache.
namespace imp {

using Variable = recipe::Variable;
using Variables = recipe::Variables;
using Args = recipe::Args;

using recipe::kDefaultOutputSocketName;

namespace {

using AsyncExecutionHandle = RecipeAsyncExecutionManager::AsyncExecutionHandle;
using ExecutionResult = RecipeRuntimeGraph::ExecutionResult;

constexpr absl::string_view kExecutionTimeExceededMessage =
    "Recipe node execution time limit exceeded.";

absl::Status CacheSocketValue(RecipeScope& scope, const NodeId& node_id,
                              absl::string_view socket_name,
                              const Variable& value) {
  std::string socket_variable_name =
      recipe::GetSocketVariableName(node_id, socket_name);

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
  return scope.GetVariable(recipe::GetSocketVariableName(node_id, socket_name));
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
      runtime_graph->event_node_map_[event_node->event_name].push_back(&node);
    }

    if (std::holds_alternative<ExecutableNode>(node.node) &&
        std::holds_alternative<CustomStatement>(
            node.executable_node()->statement)) {
      const CustomStatement* custom_statement =
          node.executable_node()->custom();
      std::unique_ptr<RecipeCustomStatement> recipe_custom_statement =
          runtime_graph->recipe_system_.CreateCustomStatement(
              custom_statement->name);
      if (!recipe_custom_statement) {
        return absl::FailedPreconditionError(absl::StrFormat(
            "Unknown CustomStatement %s", custom_statement->name));
      }

      auto [_, inserted] = runtime_graph->custom_statements_.insert(
          {node.id, std::move(recipe_custom_statement)});
      if (!inserted) {
        return absl::FailedPreconditionError(absl::StrFormat(
            "Found duplicate CustomStatement at NodeId %u", node.id.index));
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

  std::optional<BinaryExpression::BinaryOps> binary_op;
  switch (op) {
    case AssignmentStatement::AssignmentOps::ADD_ASSIGN:
      binary_op = BinaryExpression::BinaryOps::ADD;
      break;
    case AssignmentStatement::AssignmentOps::SUBTRACT_ASSIGN:
      binary_op = BinaryExpression::BinaryOps::SUBTRACT;
      break;
    case AssignmentStatement::AssignmentOps::MULTIPLY_ASSIGN:
      binary_op = BinaryExpression::BinaryOps::MULTIPLY;
      break;
    case AssignmentStatement::AssignmentOps::DIVIDE_ASSIGN:
      binary_op = BinaryExpression::BinaryOps::DIVIDE;
      break;
    case AssignmentStatement::AssignmentOps::UNKNOWN_ASSIGNMENT_OP:
      return absl::InvalidArgumentError(
          "AssignmentStatement must have a valid assignment operation.");
    case AssignmentStatement::AssignmentOps::ASSIGN:
      break;
  }

  if (binary_op.has_value()) {
    MP_ASSIGN_OR_RETURN(target, recipe::EvaluateBinaryExpression(binary_op.value(),
                                                              target, value));
  } else {
    target = value;
  }

  return absl::OkStatus();
}

}  // namespace

ExecutionResult RecipeRuntimeGraph::ExecuteAssignmentStatement(
    const AssignmentStatement& statement,
    const RecipeExecutionContext& context) const {
  // Creating a temporary scope for caching all Variables evaluated only within
  // the scope of this executable node.
  RecipeScope executable_node_scope = RecipeScope(&context.scope);
  RecipeExecutionContext executable_node_context{
      .scope = executable_node_scope,
      .view = context.view,
      .async_manager = context.async_manager};
  MP_ASSIGN_OR_RETURN(
      Variable value_result,
      EvaluateValueConnection(statement.value, executable_node_context));

  if (statement.targets.empty()) {
    return absl::InvalidArgumentError(
        "AssignmentStatement must have at least one target.");
  } else if (statement.targets.size() == 1) {
    // Single target assignment
    absl::string_view target_name = statement.targets[0].name;
    std::optional<std::reference_wrapper<Variable>> variable =
        context.scope.GetVariable(target_name);
    if (!variable) {
      return absl::NotFoundError(absl::StrFormat(
          "Can't find variable named %s to assign to.", target_name));
    }
    MP_RETURN_IF_ERROR(ExecuteAssignmentOperation(target_name, *variable,
                                               value_result, statement.op,
                                               executable_node_context.scope));
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
          context.scope.GetVariable(target_name);
      if (!variable) {
        return absl::NotFoundError(absl::StrFormat(
            "Can't find variable named %s to assign to.", target_name));
      }
      MP_RETURN_IF_ERROR(ExecuteAssignmentOperation(target_name, *variable,
                                                 tuple.values[i].value,
                                                 statement.op, context.scope));
    }
  }

  return ExecuteNode(statement.next_node, context);
}

absl::StatusOr<Variable> RecipeRuntimeGraph::EvaluateUnaryExpression(
    const UnaryExpression& expression,
    const RecipeExecutionContext& context) const {
  MP_ASSIGN_OR_RETURN(Variable input,
                   EvaluateValueConnection(expression.input, context));

  return recipe::EvaluateUnaryExpression(expression.op, input);
}

absl::StatusOr<Variable> RecipeRuntimeGraph::EvaluateBinaryExpression(
    const BinaryExpression& expression,
    const RecipeExecutionContext& context) const {
  MP_ASSIGN_OR_RETURN(Variable left_result,
                   EvaluateValueConnection(expression.left, context));

  MP_ASSIGN_OR_RETURN(Variable right_result,
                   EvaluateValueConnection(expression.right, context));

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
#endif

  imp::output::Recipe(
      "Evaluating binary_expression %s ",
      proto::EnumMetaData<BinaryExpression::BinaryOps>::GetName(expression.op));

  return recipe::EvaluateBinaryExpression(expression.op, left_result,
                                          right_result);
}

absl::StatusOr<recipe::ReturnValue> RecipeRuntimeGraph::EvaluateCallExpression(
    const CallExpression& call_expression,
    const RecipeExecutionContext& context) const {
  Args evaluated_args;
  evaluated_args.reserve(call_expression.args.size());
  for (const ValueConnection& arg_value_connection : call_expression.args) {
    MP_ASSIGN_OR_RETURN(Variable evaluated_arg,
                     EvaluateValueConnection(arg_value_connection, context));

    // TODO: Combine experimental and non-experimental code paths.
#if IMP_ENABLE_RECIPE_EXPERIMENTAL
    // Evaluating an std::monostate will coerce to the default value for the
    // type in the registered functions.
    evaluated_args.push_back(evaluated_arg);
#else
    if (absl::holds_alternative<std::monostate>(evaluated_arg)) {
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
                     EvaluateValueConnection(arg_value_connection, context));
    evaluated_named_args.push_back(std::make_pair(arg_name, evaluated_arg));
  }

  return recipe_system_.ExecuteFunction(call_expression.name, evaluated_args,
                                        evaluated_named_args);
}

ExecutionResult RecipeRuntimeGraph::ExecuteCallStatement(
    const NodeId& node_id, const CallStatement& call_statement,
    const RecipeExecutionContext& context) const {
  RecipeScope executable_node_scope{&context.scope};
  RecipeExecutionContext executable_node_context{
      .scope = executable_node_scope,
      .view = context.view,
      .async_manager = context.async_manager};
  MP_ASSIGN_OR_RETURN(recipe::ReturnValue result,
                   EvaluateCallExpression(call_statement.expression,
                                          executable_node_context));

  if (result.async_values) {
    if (!result.async_values->Ready()) {
      return absl::InternalError(absl::StrFormat(
          "Cannot call async function %s that returns unready variables with "
          "CallStatement. Use AsyncCallStatement "
          "instead.",
          call_statement.expression.name));
    } else {
      MP_ASSIGN_OR_RETURN(Variables async_values, result.async_values->Get());
      MP_RETURN_IF_ERROR(CacheSocketValues(context.scope, node_id, async_values));
    }
  }

  MP_RETURN_IF_ERROR(CacheSocketValues(context.scope, node_id, result.values));

  return ExecuteNode(call_statement.next_node, context);
}

ExecutionResult RecipeRuntimeGraph::ExecuteAsyncCallStatement(
    const NodeId& node_id, const AsyncCallStatement& call_statement,
    const RecipeExecutionContext& context) const {
  RecipeScope executable_node_scope{&context.scope};
  RecipeExecutionContext executable_node_context{
      .scope = executable_node_scope,
      .view = context.view,
      .async_manager = context.async_manager};
  MP_ASSIGN_OR_RETURN(recipe::ReturnValue result,
                   EvaluateCallExpression(call_statement.expression,
                                          executable_node_context));

  if (!result.values.empty()) {
    // Cache sync return values.
    MP_RETURN_IF_ERROR(CacheSocketValues(context.scope, node_id, result.values));
  }

  Future<absl::Status> cache_return_values_future(absl::OkStatus());
  if (result.async_values) {
    cache_return_values_future = result.async_values->Then(
        [&scope = context.scope, node_id,
         this](Variables return_values) mutable -> absl::Status {
          MP_RETURN_IF_ERROR(CacheSocketValues(scope, node_id, return_values));
          return absl::OkStatus();
        });
  }

  AsyncExecutionHandle async_handle = cache_return_values_future.Then(
      [on_done_node =
           call_statement.on_done_node]() -> ExecutableNodeConnection {
        return on_done_node;
      });

  // TODO Improve the management of RecipeScope.
  // Currently the newly added async execution is linked to the active scope of
  // the async manager, while the variable of its execution ID is published
  // to the context's member scope. We do manually make sure they are the same
  // one when executing the graph, but they should be guaranteed to always be
  // the same one by design without intervention from the programmer.
  RecipeAsyncExecutionManager::AsyncExecutionId new_execution_id =
      context.async_manager.TryAddAsyncExecution(node_id, async_handle);
  MP_RETURN_IF_ERROR(
      CacheSocketValue(context.scope, node_id,
                       std::string(recipe::kDefaultAsyncExecutionIdSocketName),
                       Variable(new_execution_id)));

  return ExecuteNode(call_statement.next_node, context);
}

ExecutionResult RecipeRuntimeGraph::ExecuteBranchStatement(
    const BranchStatement& statement,
    const RecipeExecutionContext& context) const {
  // Creating a temporary scope for caching all Variables evaluated only
  // within the scope of this executable node.
  RecipeScope executable_node_scope{&context.scope};
  RecipeExecutionContext executable_node_context{
      .scope = executable_node_scope,
      .view = context.view,
      .async_manager = context.async_manager};
  MP_ASSIGN_OR_RETURN(
      Variable test_result,
      EvaluateValueConnection(statement.test, executable_node_context));

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
    return ExecuteNode(statement.true_next_node, context);
  } else {
    return ExecuteNode(statement.false_next_node, context);
  }
#else
  if (!absl::holds_alternative<bool>(test_result)) {
    return absl::InvalidArgumentError("test must return a bool.");
  }
  if (absl::get<bool>(test_result)) {
    return ExecuteNode(statement.true_next_node, context);
  } else {
    return ExecuteNode(statement.false_next_node, context);
  }
#endif
}

ExecutionResult RecipeRuntimeGraph::ExecuteSwitchStatement(
    const SwitchStatement& statement,
    const RecipeExecutionContext& context) const {
  // Creating a temporary scope for caching all Variables evaluated only
  // within the scope of this executable node.
  RecipeScope executable_node_scope{&context.scope};
  RecipeExecutionContext executable_node_context{
      .scope = executable_node_scope,
      .view = context.view,
      .async_manager = context.async_manager};

  MP_ASSIGN_OR_RETURN(
      Variable selection,
      EvaluateValueConnection(statement.selection, executable_node_context));

  if (!absl::holds_alternative<int>(selection)) {
    return absl::InvalidArgumentError("Selection must be an integer.");
  }

  int selection_int = absl::get<int>(selection);

  auto it = statement.cases.find(selection_int);
  if (it == statement.cases.end()) {
    return ExecuteNode(statement.default_next_node, context);
  }

  return ExecuteNode(it->second, context);
}

ExecutionResult RecipeRuntimeGraph::ExecuteVariableDeclarationStatement(
    const VariableDeclarationStatement& statement,
    const RecipeExecutionContext& context) const {
  MP_RETURN_IF_ERROR(context.scope.DeclareVariable(statement.declaration));
  return ExecuteNode(statement.next_node, context);
}

ExecutionResult RecipeRuntimeGraph::ExecuteLoopStatement(
    const NodeId& node_id, const LoopStatement& statement,
    const RecipeExecutionContext& context) const {
  // Creating a temporary scope for caching all Variables evaluated only
  // within the scope of this executable node.
  RecipeScope executable_node_scope{&context.scope};
  RecipeExecutionContext executable_node_context{
      .scope = executable_node_scope,
      .view = context.view,
      .async_manager = context.async_manager};
  MP_ASSIGN_OR_RETURN(
      Variable increment,
      EvaluateValueConnection(statement.increment, executable_node_context));
  MP_ASSIGN_OR_RETURN(
      Variable start_index,
      EvaluateValueConnection(statement.start_index, executable_node_context));
  MP_ASSIGN_OR_RETURN(
      Variable end_index,
      EvaluateValueConnection(statement.end_index, executable_node_context));

  if (!absl::holds_alternative<int>(increment)) {
    return absl::InvalidArgumentError("Loop increment must be an integer.");
  }

  if (!absl::holds_alternative<int>(start_index)) {
    return absl::InvalidArgumentError("Loop start_index must be an integer.");
  }

  if (!absl::holds_alternative<int>(end_index)) {
    return absl::InvalidArgumentError("Loop end index must be an integer.");
  }

  int inc = absl::get<int>(increment);
  int start = absl::get<int>(start_index);
  int end = absl::get<int>(end_index);
  for (int i = start; i < end; i += inc) {
    // Set the "index" socket to reflect the current loop iteration.
    MP_RETURN_IF_ERROR(
        CacheSocketValue(context.scope, node_id, "index", Variable(i)));

    MP_RETURN_IF_ERROR(ExecuteNode(statement.looped_node, context));
  }

  // Set the "index" socket to the value in "end".
  MP_RETURN_IF_ERROR(
      CacheSocketValue(context.scope, node_id, "index", Variable(end)));

  return ExecuteNode(statement.completed_node, context);
}

ExecutionResult RecipeRuntimeGraph::ExecuteWhileStatement(
    const NodeId& node_id, const WhileStatement& statement,
    const RecipeExecutionContext& context) const {
  // Creating a temporary scope for caching all Variables evaluated only
  // within the scope of this executable node.
  RecipeScope executable_node_scope{&context.scope};
  RecipeExecutionContext executable_node_context{
      .scope = executable_node_scope,
      .view = context.view,
      .async_manager = context.async_manager};

  MP_ASSIGN_OR_RETURN(
      Variable condition,
      EvaluateValueConnection(statement.condition, executable_node_context));

  while (absl::holds_alternative<bool>(condition) &&
         std::get<bool>(condition)) {
    MP_RETURN_IF_ERROR(ExecuteNode(statement.looped_node, context));

    executable_node_context.scope.ClearLocalVariables();

    MP_ASSIGN_OR_RETURN(
        condition,
        EvaluateValueConnection(statement.condition, executable_node_context));
  }

  return ExecuteNode(statement.completed_node, context);
}

ExecutionResult RecipeRuntimeGraph::ExecuteSequenceStatement(
    const SequenceStatement& statement,
    const RecipeExecutionContext& context) const {
  for (const ExecutableNodeConnection& node : statement.next_nodes) {
    MP_RETURN_IF_ERROR(ExecuteNode(node, context));
  }

  return absl::OkStatus();
}

ExecutionResult RecipeRuntimeGraph::ExecuteEventTriggerStatement(
    const EventTrigger& statement,
    const RecipeExecutionContext& context) const {
  Variables event_args;
  for (const auto& [arg_name, arg] : statement.args) {
    // Creating a temporary scope for caching all Variables evaluated only
    // within the scope of this evaluation.
    RecipeScope executable_node_scope{&context.scope};
    RecipeExecutionContext executable_node_context{
        .scope = executable_node_scope,
        .view = context.view,
        .async_manager = context.async_manager};
    MP_ASSIGN_OR_RETURN(Variable evaluated_arg,
                     EvaluateValueConnection(arg, executable_node_context));
#if IMP_ENABLE_RECIPE_EXPERIMENTAL
    event_args[arg_name] = evaluated_arg;
#else
    if (absl::holds_alternative<std::monostate>(evaluated_arg)) {
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

  return ExecuteNode(statement.next_node, context);
}

absl::StatusOr<Variables> RecipeRuntimeGraph::EvaluateNode(
    const NodeId& node_id, const RecipeExecutionContext& context) const {
  MP_ASSIGN_OR_RETURN(const ValueNode* value_node, GetNode<ValueNode>(node_id));

  // If we have exceeded the execution time limit, just return.
  if (context.execution_cutoff_time.has_value() &&
      absl::Now() > context.execution_cutoff_time) {
    return absl::ResourceExhaustedError(kExecutionTimeExceededMessage);
  }

  // TODO Guard this behind IMP_RUNTIME(DEV) instead
  if constexpr (output::kEnableRecipeLog) {
    auto itr = node_map_.find(node_id);
    if (itr != node_map_.end()) {
      imp::output::Recipe("Evaluating Node %s with recipe node id %d",
                          itr->second->name, node_id.index);
    }
  }

  return std::visit(
      [this, &context](auto&& arg) -> absl::StatusOr<Variables> {
        using T = std::decay_t<decltype(arg)>;
        Variables result;
        if constexpr (std::is_same_v<T, Identifier>) {
          imp::output::Recipe("Identifier token %s.", arg.name);
          std::optional<std::reference_wrapper<Variable>> variable =
              context.scope.GetVariable(arg.name);
          if (!variable) {
            return absl::FailedPreconditionError(
                absl::StrFormat("Unable to find variable %s.", arg.name));
          }
          result[std::string(kDefaultOutputSocketName)] = *variable;
          return result;
        } else if constexpr (std::is_same_v<T, UnaryExpression>) {
          imp::output::Recipe("UnaryExpression token.");
          MP_ASSIGN_OR_RETURN(Variable return_value,
                           EvaluateUnaryExpression(arg, context));
          result[std::string(kDefaultOutputSocketName)] = return_value;
          return result;
        } else if constexpr (std::is_same_v<T, BinaryExpression>) {
          imp::output::Recipe("BinaryExpression token.");
          MP_ASSIGN_OR_RETURN(Variable return_value,
                           EvaluateBinaryExpression(arg, context));
          result[std::string(kDefaultOutputSocketName)] = return_value;
          return result;
        } else if constexpr (std::is_same_v<T, CallExpression>) {
          imp::output::Recipe("CallExpression token.");
          MP_ASSIGN_OR_RETURN(recipe::ReturnValue result,
                           EvaluateCallExpression(arg, context));
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
    const ValueConnection& value_connection,
    const RecipeExecutionContext& context) const {
  if (value_connection.literal_value()) {
    return value_connection.literal_value()->value;
  } else if (value_connection.socket_connection()) {
    return EvaluateSocketConnection(*value_connection.socket_connection(),
                                    context);
  }
  return absl::InvalidArgumentError("Unknown ValueConnection type.");
}

absl::StatusOr<Variable> RecipeRuntimeGraph::EvaluateSocketConnection(
    const SocketConnection& socket_connection,
    const RecipeExecutionContext& context) const {
  std::optional<Variable> cached_value = RetrieveCachedSocketValue(
      context.scope, socket_connection.node_id, socket_connection.socket_name);
  if (cached_value) {
    return *cached_value;
  } else if (!GetNode<ValueNode>(socket_connection.node_id).ok()) {
    return absl::InternalError(
        absl::StrFormat("Referencing an uncached socket %s on node %s",
                        socket_connection.socket_name,
                        recipe::NodeIdToString(socket_connection.node_id)));
  }

  MP_ASSIGN_OR_RETURN(Variables result,
                   EvaluateNode(socket_connection.node_id, context));

  MP_RETURN_IF_ERROR(
      CacheSocketValues(context.scope, socket_connection.node_id, result));

  auto it = result.find(std::string(socket_connection.socket_name));
  if (it == result.end()) {
    return absl::NotFoundError(absl::StrFormat("Referencing missing socket %s.",
                                               socket_connection.socket_name));
  }
  return it->second;
}

ExecutionResult RecipeRuntimeGraph::ExecuteCustomStatement(
    const ExecutableNodeConnection& connection,
    const CustomStatement& statement,
    const RecipeExecutionContext& context) const {
  NodeId node_id = *connection.node_id;

  auto it = custom_statements_.find(node_id);
  if (it == custom_statements_.end()) {
    return absl::NotFoundError(absl::StrFormat("CustomStatement not found."));
  }

  RecipeCustomStatement* custom_statement = it->second.get();

  RecipeCustomStatement::Args args = {
      .input_socket_name = connection.socket_name,
      .async_execution_manager = &context.async_manager,
      .node_id = node_id,
      .context = context,
  };

  // Creating a temporary scope for caching all Variables evaluated only
  // within the scope of this evaluation.
  RecipeScope executable_node_scope{&context.scope};
  RecipeExecutionContext executable_node_context{
      .scope = executable_node_scope,
      .view = context.view,
      .async_manager = context.async_manager};

  for (const auto& [arg_name, arg] : statement.args) {
    MP_ASSIGN_OR_RETURN(Variable evaluated_arg,
                     EvaluateValueConnection(arg, executable_node_context));
    args.variables[arg_name] = evaluated_arg;
  }

  MP_ASSIGN_OR_RETURN(RecipeCustomStatement::Result result,
                   custom_statement->Execute(args));

  if (result.async_result) {
    RecipeCustomStatement::AsyncResult& async_result = *result.async_result;

    ExecutableNodeConnection next_node_connection;
    if (async_result.next_node_socket.has_value()) {
      auto it = statement.out_connections.find(*async_result.next_node_socket);
      if (it == statement.out_connections.end()) {
        return absl::NotFoundError(
            absl::StrFormat("CustomStatement %s async execution references an "
                            "invalid socket name %s.",
                            statement.name, *async_result.next_node_socket));
      }
      next_node_connection = it->second;
    }

    AsyncExecutionHandle async_handle = async_result.values.Then(
        [&scope = context.scope, node_id, this,
         next_node_connection](recipe::Variables values) mutable
            -> absl::StatusOr<ExecutableNodeConnection> {
          MP_RETURN_IF_ERROR(CacheSocketValues(scope, node_id, values));

          return next_node_connection;
        });

    // TODO Improve the management of RecipeScope.
    // Currently the newly added async execution is linked to the active scope
    // of the async manager, while the variable of its execution ID is published
    // to the context's member scope. We do manually make sure they are the same
    // one when executing the graph, but they should be guaranteed to always be
    // the same one by design without intervention from the programmer.
    RecipeAsyncExecutionManager::AsyncExecutionId new_execution_id =
        context.async_manager.TryAddAsyncExecution(node_id, async_handle);

    std::string async_execution_id_socket_name =
        async_result.async_execution_id_socket_name.value_or(
            std::string(recipe::kDefaultAsyncExecutionIdSocketName));

    MP_RETURN_IF_ERROR(CacheSocketValue(context.scope, node_id,
                                     async_execution_id_socket_name,
                                     Variable(new_execution_id)));
  }

  RecipeCustomStatement::SyncResult& sync_result = result.sync_result;

  // Cache sync return values.
  MP_RETURN_IF_ERROR(CacheSocketValues(context.scope, *connection.node_id,
                                    sync_result.values));

  ExecutableNodeConnection next_node_connection;
  if (sync_result.next_node_socket.has_value()) {
    auto it = statement.out_connections.find(*sync_result.next_node_socket);
    if (it == statement.out_connections.end()) {
      return absl::NotFoundError(
          absl::StrFormat("CustomStatement %s sync execution references an "
                          "invalid socket name %s.",
                          statement.name, *sync_result.next_node_socket));
    }
    next_node_connection = it->second;
  }

  // Execute next node.
  return ExecuteNode(next_node_connection, context);
}

// TODO ExecutionNode currently recurses down the entire graph to
// execute the recipe graph. This does not scale well for larger graphs and we
// risk overflowing the call stack. Instead, ExecuteNode should, over a long
// execution flow, have an O(1) sized call stack.
ExecutionResult RecipeRuntimeGraph::ExecuteNode(
    const ExecutableNodeConnection& executable_node_connection,
    const RecipeExecutionContext& context) const {
  // If we have exceeded the execution time limit, just return.
  if (context.execution_cutoff_time.has_value() &&
      absl::Now() > context.execution_cutoff_time) {
    return absl::ResourceExhaustedError(kExecutionTimeExceededMessage);
  }

  if (!executable_node_connection.node_id.has_value()) {
    return absl::OkStatus();
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
      [this, &context, &node_id,
       &executable_node_connection](auto&& arg) -> ExecutionResult {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, CallStatement>) {
          imp::output::Recipe("CallStatement.");
          return ExecuteCallStatement(node_id, arg, context);
        } else if constexpr (std::is_same_v<T, AssignmentStatement>) {
          imp::output::Recipe("AssignmentStatement.");
          return ExecuteAssignmentStatement(arg, context);
        } else if constexpr (std::is_same_v<T, BranchStatement>) {
          imp::output::Recipe("BranchStatement.");
          return ExecuteBranchStatement(arg, context);
        } else if constexpr (std::is_same_v<T, VariableDeclarationStatement>) {
          imp::output::Recipe("VariableDeclarationStatement.");
          return ExecuteVariableDeclarationStatement(arg, context);
        } else if constexpr (std::is_same_v<T, LoopStatement>) {
          imp::output::Recipe("LoopStatement.");
          return ExecuteLoopStatement(node_id, arg, context);
        } else if constexpr (std::is_same_v<T, SequenceStatement>) {
          imp::output::Recipe("SequenceStatement.");
          return ExecuteSequenceStatement(arg, context);
        } else if constexpr (std::is_same_v<T, EventTrigger>) {
          imp::output::Recipe("EventTrigger.");
          return ExecuteEventTriggerStatement(arg, context);
        } else if constexpr (std::is_same_v<T, AsyncCallStatement>) {
          imp::output::Recipe("AsyncCallStatement.");
          return ExecuteAsyncCallStatement(node_id, arg, context);
        } else if constexpr (std::is_same_v<T, WhileStatement>) {
          imp::output::Recipe("WhileStatement.");
          return ExecuteWhileStatement(node_id, arg, context);
        } else if constexpr (std::is_same_v<T, SwitchStatement>) {
          imp::output::Recipe("SwitchStatement.");
          return ExecuteSwitchStatement(arg, context);
        } else if constexpr (std::is_same_v<T, CustomStatement>) {
          imp::output::Recipe("CustomStatement.");
          return ExecuteCustomStatement(executable_node_connection, arg,
                                        context);
        } else {
          return absl::NotFoundError("Unknown statement type.");
        }
      },
      executable_node->statement);
}

ExecutionResult RecipeRuntimeGraph::TriggerEvent(
    const RecipeRuntimeEvent& event,
    const RecipeExecutionContext& context) const {
  auto it = event_node_map_.find(std::string(event.name));
  if (it == event_node_map_.end()) {
    return absl::NotFoundError(
        absl::StrFormat("RecipeEvent %s not found. Skipping.", event.name));
  }

  for (const RecipeNode* node : it->second) {
    const EventNode& event_node = *node->event_node();

    MP_RETURN_IF_ERROR(
        CacheSocketValues(context.scope, node->id, event.arguments));

    MP_RETURN_IF_ERROR(ExecuteNode(event_node.next_node, context));
  }

  return absl::OkStatus();
}

ExecutionResult RecipeRuntimeGraph::ResumeExecution(
    const RecipeAsyncExecutionManager::AsyncExecution& execution,
    BaseView& view, std::optional<absl::Time> execution_cutoff_time) const {
  if (!execution.handle.Ready()) {
    return absl::FailedPreconditionError("AsyncExecutionHandle is not ready.");
  }

  MP_ASSIGN_OR_RETURN(ExecutableNodeConnection connection, execution.handle.Get());

  RecipeAsyncExecutionManager* async_manager =
      execution.async_execution_manager;

  // Resume using the scope that we scheduled this async execution with. This
  // way if more async executions are to be added during the below ExecuteNode,
  // they will be scheduled with this same scope.
  async_manager->SetActiveScope(execution.scope);

  RecipeExecutionContext context{
      .scope = *(execution.scope),
      .view = view,
      .async_manager = *async_manager,
      .execution_cutoff_time = execution_cutoff_time};

  ExecutionResult result = ExecuteNode(connection, context);

  async_manager->DeleteExecution(execution.id);

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

bool RecipeRuntimeGraph::HasEvent(absl::string_view event_name) const {
  auto it = event_node_map_.find(event_name);
  return it != event_node_map_.end();
}

}  // namespace imp
