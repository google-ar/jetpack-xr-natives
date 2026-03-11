/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_RUNTIME_GRAPH_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_RUNTIME_GRAPH_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "core/common/invocable.h"
#include "core/common/type_traits.h"
#include "core/ncsb/node.h"
#include "core/recipes/language/recipe_async_execution_manager.h"
#include "core/recipes/language/recipe_custom_statement.h"
#include "core/recipes/language/recipe_execution_context.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_runtime_event.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_system.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"

namespace imp {

// RecipeRuntimeGraph contains runtime information about a RecipeGraph and is
// able to execute or evaluate the nodes in the graph.
class RecipeRuntimeGraph {
 public:
  using ExecutionResult = absl::Status;

  static absl::StatusOr<std::unique_ptr<RecipeRuntimeGraph>> CreateRuntimeGraph(
      BaseView& view, const RecipeGraph& graph);

  // Triggers an event in the RecipeGraph.
  ExecutionResult TriggerEvent(const RecipeRuntimeEvent& event,
                               const RecipeExecutionContext& context) const;

  // Resumes an execution in the RecipeGraph and delete the async execution
  // record from the RecipeAsyncExecutionManager.
  ExecutionResult ResumeExecution(
      const RecipeAsyncExecutionManager::AsyncExecution& execution,
      BaseView& view,
      std::optional<absl::Time> execution_cutoff_time =
          std::optional<absl::Time>()) const;

  void SetRuntimeEventListener(Invocable<void(RecipeRuntimeEvent)> listener);

  void SetSocketValueListener(
      Invocable<void(const NodeId&, const recipe::Variables&)> listener);

  const RecipeGraph& GetRecipeGraph() const { return graph_; }

  bool HasEvent(absl::string_view event_name) const;

 private:
  RecipeRuntimeGraph(BaseView& view, const RecipeGraph& graph);

  // Gets a RecipeNode by its NodeId.
  // A RecipeNode can be either a ValueNode, an ExecutableNode or an EventNode.
  template <typename T>
  absl::StatusOr<const T*> GetNode(const NodeId& node_id) const {
    auto itr = node_map_.find(node_id);
    if (itr == node_map_.end()) {
      return absl::NotFoundError(absl::StrFormat(
          "Referencing missing RecipeNode at NodeId %u", node_id.index));
    }
    const RecipeNode& node = *itr->second;
    // Check if the RecipeNode contains a node of the requested type.
    if (!std::holds_alternative<T>(node.node)) {
      return absl::InvalidArgumentError(
          absl::StrFormat("Referencing invalid %s at NodeId %u",
                          imp::type_traits::kTypeName<T>, node_id.index));
    }
    return &std::get<T>(node.node);
  }

  absl::StatusOr<recipe::Variables> EvaluateNode(
      const NodeId& node_id, const RecipeExecutionContext& context) const;

  absl::StatusOr<recipe::Variable> EvaluateValueConnection(
      const ValueConnection& value_connection,
      const RecipeExecutionContext& context) const;

  absl::StatusOr<recipe::Variable> EvaluateSocketConnection(
      const SocketConnection& socket_connection,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteNode(
      const ExecutableNodeConnection& executable_node_connection,
      const RecipeExecutionContext& context) const;

  absl::StatusOr<recipe::Variable> EvaluateUnaryExpression(
      const UnaryExpression& expression,
      const RecipeExecutionContext& context) const;

  absl::StatusOr<recipe::Variable> EvaluateBinaryExpression(
      const BinaryExpression& expression,
      const RecipeExecutionContext& context) const;

  absl::StatusOr<recipe::ReturnValue> EvaluateCallExpression(
      const CallExpression& call_expression,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteAssignmentStatement(
      const AssignmentStatement& statement,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteCallStatement(
      const NodeId& node_id, const CallStatement& call_statement,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteAsyncCallStatement(
      const NodeId& node_id, const AsyncCallStatement& call_statement,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteBranchStatement(
      const BranchStatement& statement,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteSwitchStatement(
      const SwitchStatement& statement,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteVariableDeclarationStatement(
      const VariableDeclarationStatement& statement,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteWhileStatement(
      const NodeId& node_id, const WhileStatement& statement,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteLoopStatement(
      const NodeId& node_id, const LoopStatement& statement,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteSequenceStatement(
      const SequenceStatement& statement,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteEventTriggerStatement(
      const EventTrigger& statement,
      const RecipeExecutionContext& context) const;

  ExecutionResult ExecuteCustomStatement(
      const ExecutableNodeConnection& connection,
      const CustomStatement& statement,
      const RecipeExecutionContext& context) const;

  absl::Status CacheSocketValues(RecipeScope& scope, const NodeId& node_id,
                                 const recipe::Variables& socket_values) const;

  RecipeSystem& recipe_system_;

  const RecipeGraph& graph_;
  recipe::NodeIdMap<const RecipeNode*> node_map_;
  recipe::NodeIdMap<std::unique_ptr<RecipeCustomStatement>> custom_statements_;
  StringMap<std::vector<const RecipeNode*>> event_node_map_;
  Invocable<void(RecipeRuntimeEvent)> runtime_event_listener_;
  Invocable<void(const NodeId& id, const recipe::Variables&)>
      socket_value_listener_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_RUNTIME_GRAPH_H_
