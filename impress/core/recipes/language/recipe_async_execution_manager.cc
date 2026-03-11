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

#include "core/recipes/language/recipe_async_execution_manager.h"

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "core/common/log.h"
#include "absl/strings/str_format.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_scope.h"

namespace imp {

void RecipeAsyncExecutionManager::DeleteExecution(
    AsyncExecutionId execution_id) {
  for (auto& [node_id, executions_from_node] : async_execution_map_) {
    if (!executions_from_node.contains(execution_id)) {
      continue;
    }
    executions_from_node.erase(execution_id);
    --async_executions_count_;
    if (executions_from_node.empty()) {
      async_execution_map_.erase(node_id);
    }
    break;
  }
}

void RecipeAsyncExecutionManager::DeleteExecutionsScheduledByNode(
    NodeId node_id) {
  if (!async_execution_map_.contains(node_id)) {
    return;
  }
  int num_deleted = async_execution_map_[node_id].size();
  async_execution_map_.erase(node_id);
  async_executions_count_ -= num_deleted;
}

void RecipeAsyncExecutionManager::ForEachAsyncExecution(
    std::function<void(const AsyncExecution& async_execution)>&& fn) const {
  // fn could potentially delete async executions from the record, so we make a
  // temporary copy of all the async executions IDs to iterate over.
  std::vector<std::pair<NodeId, AsyncExecutionId>> to_iterate;
  for (auto& [node_id, executions_from_node] : async_execution_map_) {
    for (auto& [execution_id, execution] : executions_from_node) {
      to_iterate.emplace_back(node_id, execution_id);
    }
  }
  for (auto& [node_id, execution_id] : to_iterate) {
    if (!async_execution_map_.contains(node_id)) {
      continue;
    }
    const NodeAsyncExecutions& executions_from_node =
        async_execution_map_.at(node_id);
    if (!executions_from_node.contains(execution_id)) {
      continue;
    }
    fn(executions_from_node.at(execution_id));
  }
}

RecipeScope* RecipeAsyncExecutionManager::AddScope(RecipeScope* parent_scope) {
  AsyncExecutionScopeHolder& newly_added_scope_holder =
      scopes_.emplace_back(std::make_unique<RecipeScope>(parent_scope));
  active_scope_ = &newly_added_scope_holder;
  return newly_added_scope_holder.scope.get();
}

RecipeAsyncExecutionManager::AsyncExecutionId
RecipeAsyncExecutionManager::TryAddAsyncExecution(NodeId scheduler_node_id,
                                                  AsyncExecutionHandle handle) {
  if (active_scope_ == nullptr) {
    // This would happen if the graph execution does not have a valid active
    // scope (set by SetActiveScope) to schedule async executions with. Note
    // that AddScope() calls SetActiveScope() to set the newly added scope as
    // the active one.
    IMP_LOG(imp::FATAL) << absl::StrFormat(
        "No active scope found when Node %d was trying to add a new async "
        "execution to the record",
        scheduler_node_id.index);
  }
  AsyncExecutionId execution_id = GetNextAsyncExecutionID();

  if (!async_execution_map_.contains(scheduler_node_id)) {
    async_execution_map_[scheduler_node_id] = {};
  }

  AsyncExecutionScope scope(active_scope_);
  AsyncExecution execution = {
      .async_execution_manager = this,
      .scope = std::move(scope),
      .id = execution_id,
      .calling_node_id = scheduler_node_id,
      .handle = handle,
  };

  NodeAsyncExecutions& executions_from_node =
      async_execution_map_[scheduler_node_id];
  executions_from_node.emplace(execution_id, std::move(execution));
  ++async_executions_count_;
  return execution_id;
}

void RecipeAsyncExecutionManager::DeleteFinishedScopes() {
  auto it = scopes_.begin();
  while (it != scopes_.end()) {
    if (it->ref_counter.GetCount() == 0) {
      it = scopes_.erase(it);
    } else {
      ++it;
    }
  }
  active_scope_ = nullptr;
}

void RecipeAsyncExecutionManager::Reset() {
  async_executions_count_ = 0;
  next_async_execution_id_ = 0;
  active_scope_ = nullptr;

  async_execution_map_.clear();
  scopes_.clear();
}

RecipeAsyncExecutionManager::AsyncExecutionId
RecipeAsyncExecutionManager::GetNextAsyncExecutionID() {
  AsyncExecutionId result_id = next_async_execution_id_;
  ++next_async_execution_id_;
  return result_id;
}

}  // namespace imp
