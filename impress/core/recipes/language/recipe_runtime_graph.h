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

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/variant.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/common/type_traits.h"
#include "core/ncsb/node.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_runtime_event.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_system.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

// RecipeRuntimeGraph contains runtime information about a RecipeGraph and is
// able to execute or evaluate the nodes in the graph.
class RecipeRuntimeGraph {
 public:
  using AsyncExecutionHandle = Future<std::optional<NodeId>>;

  struct ExecutionResult {
    std::vector<AsyncExecutionHandle> async_execution_handles;

    // TODO: Optimize how ExecutionResults are combined to avoid
    // unnecessary copies.
    void Combine(const ExecutionResult& other) {
      async_execution_handles.insert(async_execution_handles.end(),
                                     other.async_execution_handles.begin(),
                                     other.async_execution_handles.end());
    }
  };

  static absl::StatusOr<std::unique_ptr<RecipeRuntimeGraph>> CreateRuntimeGraph(
      BaseView& view, const RecipeGraph& graph);

  // Triggers an event in the RecipeGraph.
  absl::StatusOr<ExecutionResult> TriggerEvent(const RecipeRuntimeEvent& event,
                                               RecipeScope* scope) const;

  // Resumes an execution in the RecipeGraph.
  absl::StatusOr<ExecutionResult> ResumeExecution(
      const AsyncExecutionHandle& handle, RecipeScope* scope) const;

  void SetRuntimeEventListener(Invocable<void(RecipeRuntimeEvent)> listener);

  void SetSocketValueListener(
      Invocable<void(const NodeId&, const recipe::Variables&)> listener);

  const RecipeGraph& GetRecipeGraph() const { return graph_; }

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

  absl::StatusOr<recipe::Variables> EvaluateNode(const NodeId& node_id,
                                                 RecipeScope& scope) const;

  absl::StatusOr<recipe::Variable> EvaluateValueConnection(
      const ValueConnection& value_connection, RecipeScope& scope) const;

  absl::StatusOr<recipe::Variable> EvaluateSocketConnection(
      const SocketConnection& socket_connection, RecipeScope& scope) const;

  absl::StatusOr<ExecutionResult> ExecuteNode(
      const ExecutableNodeConnection& executable_node_connection,
      RecipeScope& scope) const;

  absl::StatusOr<recipe::Variable> EvaluateUnaryExpression(
      const UnaryExpression& expression, RecipeScope& scope) const;

  absl::StatusOr<recipe::Variable> EvaluateBinaryExpression(
      const BinaryExpression& expression, RecipeScope& scope) const;

  absl::StatusOr<recipe::ReturnValue> EvaluateCallExpression(
      const CallExpression& call_expression, RecipeScope& scope) const;

  absl::StatusOr<ExecutionResult> ExecuteAssignmentStatement(
      const AssignmentStatement& statement, RecipeScope& scope) const;

  absl::StatusOr<ExecutionResult> ExecuteCallStatement(
      const NodeId& node_id, const CallStatement& call_statement,
      RecipeScope& scope) const;

  absl::StatusOr<ExecutionResult> ExecuteAsyncCallStatement(
      const NodeId& node_id, const AsyncCallStatement& call_statement,
      RecipeScope& scope) const;

  absl::StatusOr<ExecutionResult> ExecuteBranchStatement(
      const BranchStatement& statement, RecipeScope& scope) const;

  absl::StatusOr<ExecutionResult> ExecuteVariableDeclarationStatement(
      const VariableDeclarationStatement& statement, RecipeScope& scope) const;

  absl::StatusOr<ExecutionResult> ExecuteWhileStatement(
      const NodeId& node_id, const WhileStatement& statement,
      RecipeScope& scope) const;

  absl::StatusOr<ExecutionResult> ExecuteLoopStatement(
      const NodeId& node_id, const LoopStatement& statement,
      RecipeScope& scope) const;

  absl::StatusOr<ExecutionResult> ExecuteSequenceStatement(
      const SequenceStatement& statement, RecipeScope& scope) const;

  absl::StatusOr<ExecutionResult> ExecuteEventTriggerStatement(
      const EventTrigger& statement, RecipeScope& scope) const;

  absl::Status CacheSocketValues(RecipeScope& scope, const NodeId& node_id,
                                 const recipe::Variables& socket_values) const;

  RecipeSystem& recipe_system_;

  const RecipeGraph& graph_;
  tsl::robin_map<NodeId, const RecipeNode*, recipe::NodeIdHash,
                 recipe::NodeIdEqual>
      node_map_;
  StringMap<const RecipeNode*> event_node_map_;
  Invocable<void(RecipeRuntimeEvent)> runtime_event_listener_;
  Invocable<void(const NodeId& id, const recipe::Variables&)>
      socket_value_listener_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_RUNTIME_GRAPH_H_
