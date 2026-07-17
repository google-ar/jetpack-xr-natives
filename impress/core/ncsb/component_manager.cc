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

#include "core/ncsb/component_manager.h"

#include <cstddef>
#include <vector>

#include "filament/libs/utils/include/utils/Entity.h"
#include "core/graph/dependency_graph.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_id.h"
#include "core/view/base_view.h"

namespace imp {

ComponentManager::ComponentManager(BaseView* view) : view_(view) {}

BaseComponentPool* ComponentManager::GetComponentPoolById(
    ComponentId component_id) {
  BaseComponentPool* base_pool = nullptr;
  if (component_pools_.size() > component_id) {
    base_pool = component_pools_[component_id].get();
  }
  return base_pool;
}

UpdateSystem& ComponentManager::GetUpdateSystem() {
  return view_->GetUpdateSystem();
}

void ComponentManager::NotifyActiveForEntity(utils::Entity entity,
                                             bool active) {
  for (size_t i = 0; i < component_pools_.size(); i++) {
    BaseComponentPool* pool = component_pools_[i].get();
    if (!pool) {
      continue;
    }
    Component* component = pool->TryGetRawComponentFromEntity(entity);
    if (component != nullptr && component->IsEnabled()) {
      pool->NotifyActive(component, active);
    }
  }
}

void ComponentManager::RemoveAllFromNodes(
    const std::vector<NodeHandle>& nodes) {
  cleanup_graph_.TraverseExtras([&nodes](BaseComponentPool* pool) {
    // If the pool is null, that means no components of this type have
    // been added. That can happen here if a component was added with
    // update dependencies/dependees that haven't been added.
    if (pool) {
      for (NodeHandle node : nodes) {
        pool->Remove(node.GetEntity());
      }
    }
  });
}

void ComponentManager::DetachAll() {
  cleanup_graph_.TraverseExtras([](BaseComponentPool* pool) {
    // If the pool is null, that means no components of this type have
    // been added. That can happen here if a component was added with
    // update dependencies/dependees that haven't been added.
    if (pool) {
      pool->RemoveAll();
    }
  });
}

void ComponentManager::DestroyPools() {
  component_pools_.clear();
  cleanup_graph_ = {};
}

}  // namespace imp
