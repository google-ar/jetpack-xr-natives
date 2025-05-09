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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_PATH_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_PATH_MANAGER_H_

#include <queue>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/common/robin_set.h"
#include "core/math/mat.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp {

// PathManager enables searching all of the imp nodes.
//
// PathManager tracks parent/child relationship changes for all nodes and
// maintains a list of all root nodes.
//
// Parent/child node relationships tracked here are only updated by calls imp
// api calls. It's possible to make changes to the underlying data using direct
// calls to filament API, in which case the data here will be out of date. It's
// also possible to create entities outside of imp api.  PathManager can not
// return entities which are not imp nodes.
class PathManager {
 public:
  using RootNodeSet = RobinSet<NodeHandle>;

  explicit PathManager(BaseView* view);

  // Returns a new unordered_set of the nodes with no parents.
  inline RootNodeSet GetRootNodes() const { return RootNodeSet(root_nodes_); }

  // Adds or removes node from set of root nodes.
  void SetRoot(NodeHandle node, bool is_root);

  // Finds the first node matching the location.
  //
  // Location may be of the form "//node name" which will search for the
  // matching node name anywhere in the scene graph. Find searches using
  // preorder depth first search.  Ordering between sibling nodes is not
  // defined.
  NodeHandle Find(absl::string_view location);

  // Finds all nodes that contain the exact query string.
  // If root_node is specified, returns only descendants of that node and/or the
  // node itself if the node contains the query string.
  //
  // If the query is specified in the form of "^some string" then it will
  // search for all nodes whose names start with the given string instead.
  std::vector<NodeHandle> FindAll(absl::string_view query,
                                  NodeHandle root_node = NodeHandle());

  // Returns all descendants of a given node via depth first traversal.
  std::vector<NodeHandle> GetDescendants(NodeHandle node);

  // True if root is an ancestor of node.
  bool IsAncestorOf(NodeHandle root, NodeHandle node) const;

  // Returns the transform of the given node relative to the given root node.
  mat4f GetRelativeTransform(NodeHandle root, NodeHandle node) const;

  // Returns the transform of the given node relative to the given root node.
  mat4 GetRelativeTransformPrecise(NodeHandle root, NodeHandle node) const;

  // Returns this node or a descendant for which the predicate returns true with
  // the node.
  //
  // predicate is a function or lambda with the signature bool(NodeHandle)
  template <typename Fn>
  NodeHandle FindDescendantOrSelfIf(NodeHandle context, const Fn& predicate);

  // Returns any node that matches the predicate.  Search starts with root nodes
  // and descends to all connected children until a match is found.
  //
  // predicate is a function or lambda with the signature bool(NodeHandle)
  template <typename Fn>
  NodeHandle FindIf(const Fn& predicate);

  // Recursively searches the ancestors of a node starting from the node itself
  // for a component of type T.
  //
  // Returns the first component that is found. If no component is found,
  // returns an invalid ComponentHandle.
  template <typename T>
  ComponentHandle<T> GetComponentFromAncestorOrSelf(NodeHandle node) const;

  // Recursively searches the descendants of a node starting from the node
  // itself for components of type T.
  //
  // Returns all the components of type T.
  template <typename T>
  std::vector<ComponentHandle<T>> GetComponentsInDescendantsOrSelf(
      NodeHandle node) const;

 private:
  BaseView* view_;
  RootNodeSet root_nodes_;
};

template <typename Fn>
NodeHandle PathManager::FindDescendantOrSelfIf(NodeHandle context,
                                               const Fn& predicate) {
  if (predicate(context)) {
    return context;
  }

  for (auto child : context->GetChildren()) {
    NodeHandle result = FindDescendantOrSelfIf(child, predicate);
    if (result) {
      return result;
    }
  }
  return NodeHandle();
}

template <typename Fn>
NodeHandle PathManager::FindIf(const Fn& predicate) {
  RootNodeSet::const_iterator iter = root_nodes_.begin();
  while (iter != root_nodes_.end()) {
    NodeHandle result = FindDescendantOrSelfIf(*iter, predicate);
    if (result) {
      return result;
    }
    ++iter;
  }
  return NodeHandle();
}

template <typename T>
ComponentHandle<T> PathManager::GetComponentFromAncestorOrSelf(
    NodeHandle node) const {
  while (node) {
    ComponentHandle<T> comp = node->GetComponent<T>();
    if (comp) {
      return comp;
    }
    node = node->GetParent();
  }
  return ComponentHandle<T>();
}

template <typename T>
std::vector<ComponentHandle<T>> PathManager::GetComponentsInDescendantsOrSelf(
    NodeHandle node) const {
  std::vector<ComponentHandle<T>> components;
  std::queue<NodeHandle> search_queue;
  search_queue.push(node);

  while (!search_queue.empty()) {
    NodeHandle current_node = search_queue.front();
    search_queue.pop();

    if (!current_node) {
      continue;
    }

    ComponentHandle<T> comp = current_node->GetComponent<T>();

    if (comp) {
      components.emplace_back(comp);
    }

    // Recursively search in descendants
    std::vector<NodeHandle> descendants = current_node->GetChildren();
    for (NodeHandle descendant : descendants) {
      search_queue.push(descendant);
    }
  }

  return components;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_PATH_MANAGER_H_
