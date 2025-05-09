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

#ifndef THIRD_PARTY_IMPRESS_CORE_GRAPH_DEPENDENCY_GRAPH_H_
#define THIRD_PARTY_IMPRESS_CORE_GRAPH_DEPENDENCY_GRAPH_H_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/btree_set.h"
#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/enum_flags.h"
#include "core/graph/dependency_graph_helpers.h"

namespace imp {

// Graph for declaring dependencies between nodes of type T and then
// traversing the graph in the order of those dependencies.
//
// The graph supports both synchronous and asynchronous traversal.
//
// To traverse asynchronously, call the method ParallelTraverse. This allows
// work to be performed in parallel on background threads using futures.
//
// To traverse synchronously, call the method Traverse.
//
// It's basically a variation on a directed acyclic graph.
//
// T must be either an integral type or a type that meets these criteria:
//   - Hashable (A custom hasher type can be passed into the template params).
//   - Copyable (Preferably inexpensive to copy).
//   - Implements equality operators.
//   - Implements a ToString() method to identify the node.
//
// Optionally, a type ExtraT can be provided to associate extra data with each
// node. This extra data is not used by the DependencyGraph itself, but is
// passed to the traversal function for each node. The extra data
// can be set at any time by calling SetExtra. This is particularly useful when
// the extra data may change after the node is added to the graph. The extra
// data is copied when the graph is sorted, it should be cheap to copy.
template <typename T, typename ExtraT = NoExtra, typename Hash = absl::Hash<T>>
class DependencyGraph {
 public:
  using ParallelTraverseFn =
      typename imp_internal::DependencyGraphHelpers<T,
                                                    ExtraT>::ParallelTraverseFn;

  DependencyGraph();

  // Add a node to the graph with no dependencies.
  // Dependencies can still safely be added to this node later.
  void AddNode(T node);

  // Add a dependency to a node.
  // Automatically adds nodes that don't already exist.
  void AddDependency(T node, T dependency);

  // Traverse the  graph in order of dependencies.
  //
  // A node's dependents will not be called until the future returned by
  // ParallelTraverseFn is ready, allowing async work to be executed. If the
  // future returns an error, then the dependents will never be executed.
  //
  // It is required that ParallelTraverse is called on the main thread.
  // Async work can can be done by the function on any thread in the returned
  // future, but the function itself will always be called on the main thread.
  //
  // It is possible and safe for the async traversal to outlive the lifetime
  // of the DependencyGraph.
  //
  // A fatal error will be thrown if there is a circular dependency.
  Future<absl::Status> ParallelTraverse(const ParallelTraverseFn& fn);

  // Traverse the graph in order of dependencies.
  //
  // Fn is called for each node in the graph and take the node as a parameter.
  //
  // If the DependencyGraph has extra data associated with each node, then the
  // extra data will also be passed to the traverse function as a second
  // parameter.
  //
  // A fatal error will be thrown if there is a circular dependency.
  template <typename Fn>
  void Traverse(Fn fn);

  // Traverse the extras in the graph in order of dependencies.
  //
  // Fn is called for each extra in the graph.
  //
  // A fatal error will be thrown if there is a circular dependency.
  //
  // Unlike the other traverse methods, this method supports re-entrancy.
  template <typename Fn>
  void TraverseExtras(Fn fn);

  // Sets extra data associated with a node. This extra data will then be passed
  // to the function when the node is traversed.
  //
  // The extra can be changed at any time. It is copied when the graph is sorted
  // so it should be cheap & safe to copy.
  //
  // This is a no-op if the node does not exist in the graph.
  void SetExtra(T node, ExtraT extra);

 private:
  using Node = typename imp_internal::DependencyGraphHelpers<T, ExtraT>::Node;
  using IndicesList = std::vector<int>;
  using IndicesSet = absl::btree_set<int>;
  using Nodes = std::vector<Node>;
  using Edges = absl::flat_hash_map<int, IndicesSet>;
  using NodeToIndices = absl::flat_hash_map<T, int, Hash>;
  using Extras = std::vector<ExtraT>;
  using SortedExtrasTraversalInfo =
      typename imp_internal::DependencyGraphHelpers<
          T, ExtraT>::SortedExtrasTraversalInfo;

  struct TraversalInfo {
    int index;
    Node node;
    int num_predecessors;
    IndicesList successors;

    // Reset to zero for each call to Traverse.
    int num_traversed_predecessors;
  };
  using TraversalInfoList = std::vector<TraversalInfo>;

  enum class DirtyFlags : uint8_t {
    kSorting = (1 << 0),
    kExtras = (1 << 1),
    kSortedExtras = (1 << 2),
  };

  int GetIndex(T node);

  bool AddNodeInternal(T node);

  Future<absl::Status> ParallelRecursiveTraversal(
      const ParallelTraverseFn& fn, int index,
      TraversalInfoList* traversal_info_list);

  template <typename Fn>
  void RecursiveTraversal(Fn fn, int index,
                          TraversalInfoList* traversal_info_list);

  void TopologicalSort(IndicesList* out_leaves,
                       TraversalInfoList* out_traversal_list);

  void TopologicalSortRecursive(int index, int successor_index,
                                std::vector<bool>* seen_list,
                                std::vector<bool>* resolved_list,
                                int* resolved_count, IndicesList* out_leaves,
                                TraversalInfoList* out_traversal_list);

  void UpdateCachedState();
  void FillSortedExtras(Extras& extras);

  IndicesSet roots_;
  Nodes nodes_;
  Edges edges_;
  NodeToIndices node_to_indices_;

  // Cached state from topological sort.
  Flags<DirtyFlags> dirty_flags_;
  TraversalInfoList traversal_info_list_;
  IndicesList sorted_leaves_;

  SortedExtrasTraversalInfo sorted_extras_traversal_info_;
};

template <typename T, typename ExtraT, typename Hash>
DependencyGraph<T, ExtraT, Hash>::DependencyGraph() {
  dirty_flags_.Set(DirtyFlags::kSorting);
  dirty_flags_.Set(DirtyFlags::kSortedExtras);
}

template <typename T, typename ExtraT, typename Hash>
void DependencyGraph<T, ExtraT, Hash>::AddNode(T node) {
  if (AddNodeInternal(node)) {
    roots_.insert(GetIndex(node));
    dirty_flags_.Set(DirtyFlags::kSorting);
    dirty_flags_.Set(DirtyFlags::kSortedExtras);
  }
}

template <typename T, typename ExtraT, typename Hash>
void DependencyGraph<T, ExtraT, Hash>::AddDependency(T node, T dependency) {
  AddNode(node);

  AddNodeInternal(dependency);
  roots_.erase(GetIndex(dependency));

  edges_[GetIndex(node)].insert(GetIndex(dependency));

  dirty_flags_.Set(DirtyFlags::kSorting);
  dirty_flags_.Set(DirtyFlags::kSortedExtras);
}

template <typename T, typename ExtraT, typename Hash>
Future<absl::Status> DependencyGraph<T, ExtraT, Hash>::ParallelTraverse(
    const ParallelTraverseFn& fn) {
  UpdateCachedState();

  // Copy the traversal info because the async work may last longer than this
  // method call, and later the graph could be modified or destroyed.
  std::unique_ptr<TraversalInfoList> traversal_list_copy =
      std::make_unique<TraversalInfoList>(traversal_info_list_);

  Future<absl::Status> result(absl::OkStatus());
  for (auto index : sorted_leaves_) {
    result = result.Combine(
        ParallelRecursiveTraversal(fn, index, traversal_list_copy.get()));
  }

  // Hold onto the traversal list in memory until the traversal is finished.
  result.DependsOn(std::move(traversal_list_copy));

  return result;
}

template <typename T, typename ExtraT, typename Hash>
template <typename Fn>
void DependencyGraph<T, ExtraT, Hash>::Traverse(Fn fn) {
  UpdateCachedState();

  for (auto index : sorted_leaves_) {
    RecursiveTraversal(fn, index, &traversal_info_list_);
  }
}

template <typename T, typename ExtraT, typename Hash>
template <typename Fn>
void DependencyGraph<T, ExtraT, Hash>::TraverseExtras(Fn fn) {
  if constexpr (!std::is_same_v<ExtraT, NoExtra>) {
    if (!sorted_extras_traversal_info_.is_traversing_extras) {
      // Non-reentrant case.

      sorted_extras_traversal_info_.is_traversing_extras = true;

      // Update the sorted extras if they are dirty.
      if (dirty_flags_.Test(DirtyFlags::kSortedExtras)) {
        FillSortedExtras(sorted_extras_traversal_info_.sorted_extras);
        dirty_flags_.Set(DirtyFlags::kSortedExtras, false);
      }

      for (ExtraT& extra : sorted_extras_traversal_info_.sorted_extras) {
        fn(extra);
      }

      sorted_extras_traversal_info_.is_traversing_extras = false;
    } else if (!dirty_flags_.Test(DirtyFlags::kSortedExtras)) {
      // Reentrant case & the sorted extras are not dirty, can just use the
      // cached sorted extras.

      for (ExtraT& extra : sorted_extras_traversal_info_.sorted_extras) {
        fn(extra);
      }
    } else {
      // Rare reentrant case where the sorted extras are dirty.

      // Determine new sorted extras in a local variable without storing
      // them in sorted_extras_ so it doesn't invalidate the iterator.
      Extras extras;
      FillSortedExtras(extras);

      for (ExtraT& extra : extras) {
        fn(extra);
      }
    }
  }
}

template <typename T, typename ExtraT, typename Hash>
void DependencyGraph<T, ExtraT, Hash>::SetExtra(T node, ExtraT extra) {
  auto itr = node_to_indices_.find(node);
  if (itr == node_to_indices_.end()) {
    return;
  }

  nodes_[itr->second].extra = std::move(extra);
  dirty_flags_.Set(DirtyFlags::kExtras);
  dirty_flags_.Set(DirtyFlags::kSortedExtras);
}

template <typename T, typename ExtraT, typename Hash>
int DependencyGraph<T, ExtraT, Hash>::GetIndex(T node) {
  return node_to_indices_[node];
}

template <typename T, typename ExtraT, typename Hash>
bool DependencyGraph<T, ExtraT, Hash>::AddNodeInternal(T node) {
  auto itr = node_to_indices_.find(node);
  if (itr != node_to_indices_.end()) {
    return false;
  }

  nodes_.push_back({node});
  node_to_indices_[node] = nodes_.size() - 1;
  return true;
}

template <typename T, typename ExtraT, typename Hash>
Future<absl::Status>
DependencyGraph<T, ExtraT, Hash>::ParallelRecursiveTraversal(
    const ParallelTraverseFn& fn, int index,
    TraversalInfoList* traversal_info_list) {
  TraversalInfo& traversal_info = (*traversal_info_list)[index];

  traversal_info.num_traversed_predecessors++;

  if (traversal_info.num_traversed_predecessors <
      traversal_info.num_predecessors) {
    return Future<absl::Status>(absl::OkStatus());
  }

  return imp_internal::DependencyGraphHelpers<
             T, ExtraT>::InvokeParallelTraverseFn(fn, traversal_info.node)
      .Then(
          [=](const absl::Status& status) {
            if (!status.ok()) {
              return Future<absl::Status>(status);
            }
            TraversalInfo& traversal_info = (*traversal_info_list)[index];

            Future<absl::Status> successors_future(absl::OkStatus());
            for (auto successor : traversal_info.successors) {
              successors_future =
                  successors_future.Combine(ParallelRecursiveTraversal(
                      fn, successor, traversal_info_list));
            }

            return successors_future;
          },
          Executor::Type::kForeground);
}

template <typename T, typename ExtraT, typename Hash>
template <typename Fn>
void DependencyGraph<T, ExtraT, Hash>::RecursiveTraversal(
    Fn fn, int index, TraversalInfoList* traversal_info_list) {
  TraversalInfo& traversal_info = (*traversal_info_list)[index];

  traversal_info.num_traversed_predecessors++;

  if (traversal_info.num_traversed_predecessors <
      traversal_info.num_predecessors) {
    return;
  }

  imp_internal::DependencyGraphHelpers<T, ExtraT>::InvokeTraverseFn(
      fn, traversal_info.node);

  for (auto successor : traversal_info.successors) {
    RecursiveTraversal(fn, successor, traversal_info_list);
  }
}

template <typename T, typename ExtraT, typename Hash>
void DependencyGraph<T, ExtraT, Hash>::TopologicalSort(
    IndicesList* out_leaves, TraversalInfoList* out_traversal_list) {
  std::vector<bool> resolved_list(nodes_.size());
  std::vector<bool> seen_list(nodes_.size());
  int resolved_count = 0;

  for (auto root_index : roots_) {
    TopologicalSortRecursive(root_index, -1, &seen_list, &resolved_list,
                             &resolved_count, out_leaves, out_traversal_list);
  }

  // If we sorted fewer nodes than there are in the nodes_ list, that means
  // that there are orphaned cycles of nodes. For example,
  // if 1 depends on 2, and 2 depends on 1, then neither node will be in the
  // roots_ set and won't be sorted.
  if (resolved_count < nodes_.size()) {
    std::string nodes_in_cycle_string;
    for (int i = 0; i < nodes_.size(); i++) {
      if (!resolved_list[i]) {
        std::string node_string =
            imp_internal::DependencyGraphHelpers<T, ExtraT>::ToString(
                nodes_[i]);

        nodes_in_cycle_string.append(node_string);
        if (i != nodes_.size() - 1) {
          nodes_in_cycle_string.append(", ");
        }
      }
    }

    IMP_LOG(imp::FATAL) << "Circular dependency in graph detected at nodes "
               << nodes_in_cycle_string;
  }
}

template <typename T, typename ExtraT, typename Hash>
void DependencyGraph<T, ExtraT, Hash>::TopologicalSortRecursive(
    int index, int successor_index, std::vector<bool>* seen_list,
    std::vector<bool>* resolved_list, int* resolved_count,
    IndicesList* out_leaves, TraversalInfoList* out_traversal_list) {
  (*seen_list)[index] = true;

  auto edges_itr = edges_.find(index);
  if (edges_itr != edges_.end()) {
    for (auto edge : edges_itr->second) {
      if (!resolved_list->at(edge)) {
        if (seen_list->at(edge)) {
          std::string node_string =
              imp_internal::DependencyGraphHelpers<T, ExtraT>::ToString(
                  nodes_[index]);
          IMP_LOG(imp::FATAL) << "Circular dependency in graph detected at node "
                     << node_string;
        }

        TopologicalSortRecursive(edge, index, seen_list, resolved_list,
                                 resolved_count, out_leaves,
                                 out_traversal_list);
      } else {
        out_traversal_list->at(edge).successors.push_back(index);
      }
    }
  }

  (*resolved_list)[index] = true;
  ++(*resolved_count);

  int num_edges = edges_itr != edges_.end() ? edges_itr->second.size() : 0;
  if (num_edges == 0) {
    out_leaves->push_back(index);
  }

  TraversalInfo& traversal_info = out_traversal_list->at(index);
  traversal_info.index = index;
  traversal_info.node = nodes_[index];
  traversal_info.successors.clear();
  if (successor_index != -1) {
    traversal_info.successors.push_back(successor_index);
  }
  traversal_info.num_predecessors = num_edges;
  traversal_info.num_traversed_predecessors = 0;
}

template <typename T, typename ExtraT, typename Hash>
void DependencyGraph<T, ExtraT, Hash>::UpdateCachedState() {
  if (dirty_flags_.Test(DirtyFlags::kSorting)) {
    sorted_leaves_.clear();
    traversal_info_list_.resize(nodes_.size());
    TopologicalSort(&sorted_leaves_, &traversal_info_list_);
    dirty_flags_.Set(DirtyFlags::kSorting, false);
    // Extras are also updated upon sorting.
    dirty_flags_.Set(DirtyFlags::kExtras, false);
  } else {
    for (auto& traversal_info : traversal_info_list_) {
      traversal_info.num_traversed_predecessors = 0;
    }

    // Not as common to update the extras without sorting.
    if constexpr (!std::is_same_v<ExtraT, NoExtra>) {
      if (dirty_flags_.Test(DirtyFlags::kExtras)) {
        for (auto& traversal_info : traversal_info_list_) {
          // Update the node from the nodes_ list which will copy the extra
          // over.
          traversal_info.node = nodes_[traversal_info.index];
        }
      }
      dirty_flags_.Set(DirtyFlags::kExtras, false);
    }
  }
}

template <typename T, typename ExtraT, typename Hash>
void DependencyGraph<T, ExtraT, Hash>::FillSortedExtras(Extras& extras) {
  extras.clear();
  extras.reserve(nodes_.size());
  Traverse([&extras](T node, ExtraT extra) { extras.push_back(extra); });
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GRAPH_DEPENDENCY_GRAPH_H_
