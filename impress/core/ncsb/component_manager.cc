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
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/compact_bitset.h"
#include "core/graph/dependency_graph.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/node_children_iterator.h"
#include "core/ncsb/node_controller.h"
#include "core/view/base_view.h"

namespace imp {

constexpr size_t kReservedScratchSpaceCapacity = 128;

ComponentManager::ComponentManager(BaseView* view) : view_(view) {
  // Reserve space to avoid allocations in DestroySubtreeComponents.
  cleanup_ranks_.reserve(kReservedScratchSpaceCapacity);
  counts_scratch_.reserve(kReservedScratchSpaceCapacity);
  offsets_scratch_.reserve(kReservedScratchSpaceCapacity);
  ordered_pairs_scratch_.reserve(kReservedScratchSpaceCapacity);
}

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

void ComponentManager::NotifyActiveForEntity(
    utils::Entity entity,
    imp_internal::NodeController::ComponentBitset component_bitset,
    bool active) {
  component_bitset.ForEachSetBit([this, entity, active](size_t component_id) {
    BaseComponentPool* pool = component_pools_[component_id].get();
    Component* component = pool->TryGetRawComponentFromEntity(entity);
    if (component && component->IsEnabled()) {
      pool->NotifyActive(component, active);
    }
  });
}

void ComponentManager::UpdateCleanupRanks() {
  if (!cleanup_ranks_.empty()) {
    // Ranks have already been calculated.
    return;
  }

  cleanup_ranks_.resize(component_pools_.size());
  int16_t rank = 0;
  cleanup_graph_.TraverseExtras([this, &rank](BaseComponentPool* pool) {
    if (pool) {
      ComponentId id = pool->GetComponentId();
      cleanup_ranks_[id] = rank++;
    }
  });
}

void ComponentManager::DestroySubtreeComponents(
    NodeHandle root, std::vector<NodeHandle>& out_nodes) {
  // Ensure the cleanup ranks are up to date.
  UpdateCleanupRanks();

  // Track the starting index of nodes added in this call.
  //
  // This is important to allow for recursive calls when destroying nodes.
  size_t nodes_start_index = out_nodes.size();

  // Counting Sort Algorithm:
  //
  // Optimally sorts components scheduled for destruction based on their
  // topological cleanup ranks, guaranteeing consistent dependency ordering.

  // Step 1: Count and Gather Nodes
  //
  // Traverse the entire subtree in post-order depth-first order to collect
  // all descendants. Simultaneously, tally the total number of components
  // belonging to each cleanup rank.
  counts_scratch_.assign(cleanup_ranks_.size(), 0);
  auto count_fn = [this, &out_nodes](auto&& self, NodeHandle node) -> void {
    NodeChildrenRange children_range = node->GetChildrenRange();
    for (auto child : children_range) {
      self(self, child);
    }

    node->node_controller_->GetComponentBitset().ForEachSetBit(
        [this](size_t id) {
          int16_t rank = cleanup_ranks_[id];
          counts_scratch_[rank]++;
        });
    out_nodes.push_back(node);
  };

  count_fn(count_fn, root);

  // Step 2: Calculate offsets for each rank
  //
  // Transform the counts into prefix sums. These act as starting indices for
  // grouping components of the same cleanup rank continuously in the final
  // sorted array.
  offsets_scratch_.assign(cleanup_ranks_.size(), 0);
  for (size_t i = 1; i < cleanup_ranks_.size(); ++i) {
    offsets_scratch_[i] = offsets_scratch_[i - 1] + counts_scratch_[i - 1];
  }

  // Step 3: Scatter into ordered_pairs based on rank
  //
  // Place every {ComponentId, NodeHandle} tuple directly into its correct
  // position using the pre-computed offsets, automatically ordering components
  // from lowest cleanup rank to highest.
  size_t total_components =
      offsets_scratch_.empty()
          ? 0
          : (offsets_scratch_.back() + counts_scratch_.back());

  // Track the starting index for components to be destroyed in this
  // call. This allows for recursive calls to DestroySubtreeComponents to
  // correctly place components in the ordered_pairs_scratch_ vector.
  size_t start_index = ordered_pairs_scratch_.size();
  ordered_pairs_scratch_.resize(start_index + total_components);

  for (size_t i = nodes_start_index; i < out_nodes.size(); ++i) {
    NodeHandle node = out_nodes[i];
    node->node_controller_->GetComponentBitset().ForEachSetBit(
        [this, node, start_index](size_t id) {
          int16_t rank = cleanup_ranks_[id];
          size_t idx = offsets_scratch_[rank]++;
          ordered_pairs_scratch_[start_index + idx] = {id, node};
        });
  }

  // Step 4: Destroy components in order
  //
  // Sequentially remove components. Because they are sorted by rank, any
  // component that depends on another during cleanup will reliably be destroyed
  // before its dependency.
  ComponentId current_id = std::numeric_limits<ComponentId>::max();
  BaseComponentPool* current_pool = nullptr;

  for (size_t i = 0; i < total_components; ++i) {
    auto [id, node] = ordered_pairs_scratch_[start_index + i];

    if (id != current_id) {
      current_id = id;
      current_pool = component_pools_[current_id].get();
    }

    if (current_pool) {
      current_pool->Remove(node.GetEntity());
    }
  }

  // Clear the scratch space that was used for this call to
  // DestroySubtreeComponents.
  ordered_pairs_scratch_.resize(start_index);

  // Return the scratch space to kReservedScratchSpaceCapacity capacity if it's
  // larger to prevent unbounded growth when destroying a gigantic tree.
  //
  // This is only needed for ordered_pairs_scratch_ because the other scratch
  // vectors are inheritently bound to the number of component types.
  //
  // The empty() check ensures this is the top-level call to
  // DestroySubtreeComponents.
  if (ordered_pairs_scratch_.empty() &&
      ordered_pairs_scratch_.capacity() > kReservedScratchSpaceCapacity) {
    ordered_pairs_scratch_ = std::vector<std::pair<ComponentId, NodeHandle>>();
    ordered_pairs_scratch_.reserve(kReservedScratchSpaceCapacity);
  }
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
  cleanup_ranks_.clear();
}

}  // namespace imp
