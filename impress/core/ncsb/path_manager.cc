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

#include "core/ncsb/path_manager.h"

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/match.h"
#include "absl/strings/string_view.h"
#include "core/math/mat.h"
#include "core/ncsb/node.h"
#include "core/view/base_view.h"

namespace imp {
namespace {
constexpr absl::string_view kDoubleSlash = "//";
constexpr absl::string_view kCaret = "^";
}  // namespace

PathManager::PathManager(BaseView* view) : view_(view) {}

void PathManager::SetRoot(NodeHandle node, bool is_root) {
  bool was_root = root_nodes_.find(node) != root_nodes_.end();
  if (was_root == is_root) {
    return;
  }
  if (is_root) {
    root_nodes_.insert(node);
  } else {
    root_nodes_.erase(node);
  }
}

NodeHandle PathManager::Find(absl::string_view location) {
  if (location.empty()) {
    return NodeHandle();
  }
  if (absl::StartsWith(location, kDoubleSlash)) {
    absl::string_view node_name = location.substr(kDoubleSlash.size());
    return FindIf(
        [node_name](NodeHandle node) { return node->GetName() == node_name; });
  }
  return NodeHandle();
}

std::vector<NodeHandle> PathManager::FindAll(absl::string_view query,
                                             NodeHandle root_node) {
  if (query.empty()) {
    return std::vector<NodeHandle>();
  }

  std::vector<NodeHandle> result;
  bool has_double_slash = false;
  if (absl::StartsWith(query, kCaret)) {
    query = query.substr(kCaret.size());
    has_double_slash = true;
  }

  auto find_nodes_by_name = [&result, query,
                             has_double_slash](NodeHandle node) {
    if (node->GetName().length() >= query.length() &&
        (has_double_slash ? absl::StartsWith(node->GetName(), query)
                          : absl::StrContains(node->GetName(), query))) {
      result.push_back(node);
    }
    // Returning false on all tests of this predicate so that FindIf will
    // traverse the entire graph
    return false;
  };

  if (root_node) {
    FindDescendantOrSelfIf(root_node, find_nodes_by_name);
  } else {
    FindIf(find_nodes_by_name);
  }
  return result;
}

std::vector<NodeHandle> PathManager::GetDescendants(NodeHandle node) {
  std::vector<NodeHandle> children = node->GetChildren();
  if (children.empty()) {
    return {};
  }

  std::vector<NodeHandle> result;
  for (NodeHandle child : children) {
    std::vector<NodeHandle> sub_result = GetDescendants(child);
    result.insert(result.end(), sub_result.begin(), sub_result.end());
  }

  result.insert(result.end(), children.begin(), children.end());
  return result;
}

bool PathManager::IsAncestorOf(NodeHandle root, NodeHandle node) const {
  NodeHandle ancestor = node;
  while (ancestor.IsValid()) {
    if (ancestor == root) {
      return true;
    }
    ancestor = ancestor->GetParent();
  }
  return false;
}

mat4f PathManager::GetRelativeTransform(NodeHandle root,
                                        NodeHandle node) const {
  return inverse(root->GetWorldTrs()) * node->GetWorldTrs();
}

mat4 PathManager::GetRelativeTransformPrecise(NodeHandle root,
                                              NodeHandle node) const {
  return inverse(root->GetWorldTrsPrecise()) * node->GetWorldTrsPrecise();
}

}  // namespace imp
