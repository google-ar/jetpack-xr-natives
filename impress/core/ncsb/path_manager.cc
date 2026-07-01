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

#include "absl/strings/match.h"
#include "absl/strings/string_view.h"
#include "core/math/mat.h"
#include "core/ncsb/node.h"
#include "core/view/base_view.h"

namespace imp {
namespace {
constexpr absl::string_view kDoubleSlash = "//";
constexpr absl::string_view kCaret = "^";

void GetDescendantsInternal(NodeHandle node, std::vector<NodeHandle>& result) {
  auto children = node->GetChildrenRange();
  for (NodeHandle child : children) {
    GetDescendantsInternal(child, result);
  }
  for (NodeHandle child : children) {
    result.push_back(child);
  }
}

void FindAllInternal(NodeHandle node, absl::string_view query,
                     bool starts_with_caret, std::vector<NodeHandle>& result) {
  if (starts_with_caret) {
    if (absl::StartsWith(node->GetName(), query)) {
      result.push_back(node);
    }
  } else {
    if (absl::StrContains(node->GetName(), query)) {
      result.push_back(node);
    }
  }
  for (NodeHandle child : node->GetChildrenRange()) {
    FindAllInternal(child, query, starts_with_caret, result);
  }
}

}  // namespace

PathManager::PathManager(BaseView* view) : view_(view) {}

void PathManager::SetRoot(NodeHandle node, bool is_root) {
  if (node->IsRoot() == is_root) {
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
  bool starts_with_caret = false;
  if (absl::StartsWith(query, kCaret)) {
    query = query.substr(kCaret.size());
    starts_with_caret = true;
  }

  if (root_node) {
    FindAllInternal(root_node, query, starts_with_caret, result);
  } else {
    for (NodeHandle root : root_nodes_) {
      FindAllInternal(root, query, starts_with_caret, result);
    }
  }
  return result;
}

std::vector<NodeHandle> PathManager::GetDescendants(NodeHandle node) {
  std::vector<NodeHandle> result;
  GetDescendantsInternal(node, result);
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
