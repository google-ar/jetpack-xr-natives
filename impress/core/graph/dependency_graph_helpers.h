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

#ifndef THIRD_PARTY_IMPRESS_CORE_GRAPH_DEPENDENCY_GRAPH_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_GRAPH_DEPENDENCY_GRAPH_HELPERS_H_

#include <functional>
#include <string>
#include <type_traits>
#include <vector>

#include "absl/status/status.h"
#include "core/async/future.h"

namespace imp {

// Empty struct used to indicate that the DependencyGraph should not have any
// extra data associated with each node.
struct NoExtra {};

namespace imp_internal {

// Helper for DependencyGraph used when there is extra data associated with
// each node.
template <typename T, typename ExtraT>
struct DependencyGraphHelpers {
  // Parallel traverse function take the node and the extra data associated with
  // it.
  using ParallelTraverseFn =
      std::function<imp::Future<absl::Status>(T, ExtraT)>;
  using Extras = std::vector<ExtraT>;

  // Node is a struct that contains both the value of type T and the extra data.
  struct Node {
    T value;

    // Stores the extra data associated with this node.
    //
    // It's important to explicitly initialize this to the default value of
    // ExtraT so that the value is not garbage if ExtraT is a primitive or
    // pointer.
    ExtraT extra = {};
  };

  // Stores extra info that is used for TraverseExtras.
  struct SortedExtrasTraversalInfo {
    // List of fully sorted extras so that they can be efficiently traversed
    // in-order. This is used to avoid the overhead of traversing with the
    // normal traversal data structure, and makes it easier to handle
    // re-entrancy.
    Extras sorted_extras;

    // Used to track if we are calling TraverseExtras in a reentrant manner.
    bool is_traversing_extras = false;
  };

  // Invoke the traverse function for the given node.
  template <typename Fn>
  inline static void InvokeTraverseFn(Fn fn, const Node& node) {
    fn(node.value, node.extra);
  }

  // Invoke the parallel traverse function for the given node.
  inline static imp::Future<absl::Status> InvokeParallelTraverseFn(
      const ParallelTraverseFn& fn, const Node& node) {
    return fn(node.value, node.extra);
  }

  // Returns a string representing the node for debugging.
  static std::string ToString(const Node& node) {
    if constexpr (std::is_integral<T>::value) {
      return std::to_string(node.value);
    } else {
      return node.value.ToString();
    }
  }
};

// Partial specialization of DependencyGraphHelpers for when there is no extra
// data associated with each node.
template <typename T>
struct DependencyGraphHelpers<T, imp::NoExtra> {
  // Parallel traverse functions take only the node.
  using ParallelTraverseFn = std::function<imp::Future<absl::Status>(T)>;

  // Node is just the value of type T itself.
  using Node = T;

  // Empty placeholder for when there is no extra data associated with each
  // node.
  struct SortedExtrasTraversalInfo {};

  // Invoke the traverse function for the given node.
  template <typename Fn>
  inline static void InvokeTraverseFn(Fn fn, const Node& node) {
    fn(node);
  }

  // Invoke the parallel traverse function for the given node.
  inline static imp::Future<absl::Status> InvokeParallelTraverseFn(
      const ParallelTraverseFn& fn, const Node& node) {
    return fn(node);
  }

  // Returns a string representing the node for debugging.
  static std::string ToString(const Node& node) {
    if constexpr (std::is_integral<T>::value) {
      return std::to_string(node);
    } else {
      return node.ToString();
    }
  }
};

}  // namespace imp_internal
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GRAPH_DEPENDENCY_GRAPH_HELPERS_H_
