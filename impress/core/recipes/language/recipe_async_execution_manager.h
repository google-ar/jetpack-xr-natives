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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_RUNTIME_ASYNC_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_RUNTIME_ASYNC_SYSTEM_H_

#include "core/common/ref_counter.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp {

// RecipeAsyncExecutionManager manages the record of async executions during
// runtime. Each async execution has a globally unique ID. One can use this
// class to add/delete async executions and consume previously scheduled async
// executions on record.
class RecipeAsyncExecutionManager {
 public:
  using AsyncExecutionHandle = Future<ExecutableNodeConnection>;
  using AsyncExecutionId = int;

  struct AsyncExecutionScopeHolder {
    AsyncExecutionScopeHolder(std::unique_ptr<RecipeScope> scope)
        : scope(std::move(scope)), ref_counter({}) {}
    std::unique_ptr<RecipeScope> scope;
    RefCounter ref_counter;
  };

  class AsyncExecutionScope {
   public:
    RecipeScope* operator->() const { return holder_->scope.get(); }
    RecipeScope* operator()() const { return holder_->scope.get(); }
    RecipeScope& operator*() const { return *(holder_->scope); }

    int GetRefCount() const { return ref_.GetCount(); }

    AsyncExecutionScopeHolder* GetHolder() const { return holder_; }

   private:
    AsyncExecutionScope(AsyncExecutionScopeHolder* holder)
        : holder_(holder), ref_(holder->ref_counter.Retain()) {}
    AsyncExecutionScopeHolder* holder_;
    RefCounter::Ref ref_;

    friend class RecipeAsyncExecutionManager;
  };

  struct AsyncExecution {
    RecipeAsyncExecutionManager* async_execution_manager;
    AsyncExecutionScope scope;
    AsyncExecutionId id;
    NodeId calling_node_id;
    AsyncExecutionHandle handle;
  };

  // Delete an async execution from the record, using its unique ID.
  void DeleteExecution(AsyncExecutionId execution_id);

  // Delete all async executions scheduled by a certain node from the record.
  void DeleteExecutionsScheduledByNode(NodeId node_id);

  // An accessor for each async execution on record.
  void ForEachAsyncExecution(
      std::function<void(const AsyncExecution&)>&& fn) const;

  // Generate a new RecipeScope based on the parent scope, and this
  // RecipeAsyncExecutionManager is its sole owner. Set this new scope as the
  // active one so all subsequently scheduled async executions will reference
  // it (until the active scope is changed). Return a pointer to this newly
  // added RecipeScope.
  RecipeScope* AddScope(RecipeScope* parent_scope);

  // Try to add a new async execution to the record. Returns the ID of the new
  // async execution if successful, otherwise returns the error status.
  AsyncExecutionId TryAddAsyncExecution(NodeId scheduler_node_id,
                                        AsyncExecutionHandle handle);

  // Set the active scope to be the scope referenced by a certain async
  // execution
  void SetActiveScope(const AsyncExecutionScope& scope) {
    active_scope_ = scope.GetHolder();
  }

  // Iterate through all scopes on record and delete those that are not
  // referenced by any async executions.
  void DeleteFinishedScopes();

  // Resets the state of the async execution manager and cancels all async
  // executions in flight.
  void Reset();

 private:
  AsyncExecutionId GetNextAsyncExecutionID();

  using NodeAsyncExecutions =
      absl::flat_hash_map<AsyncExecutionId, AsyncExecution>;

  recipe::NodeIdMap<NodeAsyncExecutions> async_execution_map_;

  std::list<AsyncExecutionScopeHolder> scopes_;
  AsyncExecutionScopeHolder* active_scope_ = nullptr;

  AsyncExecutionId next_async_execution_id_ = 0;
  int async_executions_count_ = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_RUNTIME_ASYNC_SYSTEM_H_
