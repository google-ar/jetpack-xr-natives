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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_CUSTOM_STATEMENT_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_CUSTOM_STATEMENT_H_

#include <optional>
#include <string>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/recipes/language/recipe_async_execution_manager.h"
#include "core/recipes/language/recipe_execution_context.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp {

// RecipeCustomStatement is the base class for custom statements that are
// executed by the RecipeRuntimeGraph. Similar to RecipeRuntimeGraph,
// RecipeCustomStatement is intended to be the runtime instance of a
// CustomStatement.
//
// RecipeCustomStatements are registered to the RecipeSystem and are executed
// by the RecipeRuntimeGraph when a RecipeGraph contains a CustomStatement node.
//
// RecipeCustomStatements can have both sync and async results.
//
// RecipeCustomStatements reference the next nodes to execute by socket names.
// RecipeRuntimeGraph will look up the socket name in the CustomStatement node's
// out_connections to determine which node to execute next.
//
// RecipeCustomStatements are identified by a static constant named `kName`. For
// example:
//   class MyCustomStatement : public RecipeCustomStatement {
//    public:
//     static constexpr absl::string_view kName = "MyCustomStatementName";
//   };
// MyCustomStatement::kName is then used for registering and looking up the
// CustomStatement type in the RecipeSystem and RecipeRuntimeGraph.
class RecipeCustomStatement {
 public:
  virtual ~RecipeCustomStatement() = default;

  // Args contains the arguments passed into the CustomStatement node.
  struct Args {
    recipe::Variables variables;
    // The socket name of the incoming ExecutableNodeConnection to the
    // CustomStatement.
    std::optional<std::string> input_socket_name;
    // The async execution manager that we can use to manipulate the async
    // executions on record.
    RecipeAsyncExecutionManager* async_execution_manager;
    // The node ID of this CustomStatement node.
    NodeId node_id;
    // The execution context of the CustomStatement.
    RecipeExecutionContext context;
  };

  // SyncResult contains the result of the CustomStatement that is ready to
  // execute immediately.
  //
  // Next node connection is optional to account for the case where the
  // CustomStatement does not need to execute the next node.
  struct SyncResult {
    recipe::Variables values;
    // The socket name of the next node to execute.
    std::optional<std::string> next_node_socket;
  };

  // AsyncResult contains values that are returned through a Future.
  //
  // Next node connection is also optional to account for the case where the
  // CustomStatement does not need to execute the next node.
  struct AsyncResult {
    Future<recipe::Variables> values;
    // The socket name of the async execution ID.
    std::optional<std::string> async_execution_id_socket_name;
    // The socket name of the next node to execute.
    std::optional<std::string> next_node_socket;
  };

  // Result contains the sync and async results of the CustomStatement.
  //
  // While sync_result is always present, async_result is only present if the
  // CustomStatement schedules async execution.
  struct Result {
    SyncResult sync_result;
    std::optional<AsyncResult> async_result;
  };

  // Executes the CustomStatement and returns the result.
  virtual absl::StatusOr<Result> Execute(const Args& args) = 0;

 protected:
  // Finds and obtains the actual of value of a variable in a given StringMap of
  // Variables, or returns std::nullopt if otherwise.
  template <typename T>
  std::optional<T> GetVariable(recipe::Variables variables,
                               absl::string_view variable_name) {
    recipe::Variables::iterator it = variables.find(variable_name);
    if (it == variables.end() || !absl::holds_alternative<T>(it->second)) {
      return std::nullopt;
    }

    return std::get<T>(it->second);
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_CUSTOM_STATEMENT_H_
