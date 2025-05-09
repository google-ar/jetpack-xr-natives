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

#include "core/ncsb/groups_manager.h"

#include <cstddef>
#include <functional>
#include <string>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Scene.h"
#include "core/common/hash.h"
#include "core/common/platform_helpers.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"

namespace imp {

GroupsManager::GroupsManager(BaseView* view, filament::Scene* main_scene)
    : view_(view) {
  groups_.insert_or_assign(kMainGroupHash, Group{.scene = main_scene});
  hashes_to_names_.insert_or_assign(kMainGroupHash,
                                    std::string(kMainGroupName));
}

void GroupsManager::AddNodeToGroup(absl::string_view group_name,
                                   HashValue group_hash, NodeHandle node) {
  Group* group = nullptr;
  auto itr = groups_.find(group_hash);
  if (itr != groups_.end()) {
    group = &itr->second;
  } else {
    filament::Scene* scene = BaseView::GetSharedEngine()->createScene();
    auto emplace_result =
        groups_.insert_or_assign(group_hash, Group{.scene = scene});
    hashes_to_names_.insert_or_assign(group_hash, std::string(group_name));
    group = &emplace_result.first->second;
    view_->GetDispatcher().Send(GroupCreatedEvent(group_name));
  }

  group->num_nodes_in_layer++;

  // Only actually add the node to the scene if it is currently active.
  // Nodes internally track which groups they are part of. When the node becomes
  // active, it will be added to the scene when SetNodeActiveInGroup is called.
  if (node->IsActive()) {
    group->scene->addEntity(node.GetEntity());
  }
}

void GroupsManager::RemoveNodeFromGroup(HashValue group_hash, NodeHandle node) {
  auto itr = groups_.find(group_hash);
  if (itr == groups_.end()) {
    IMP_LOG(imp::FATAL) << "Attempting to remove node from a group that doesn't exist.";
    return;
  }

  Group& group = itr->second;
  group.num_nodes_in_layer--;
  group.scene->remove(node.GetEntity());
  if (group.num_nodes_in_layer == 0 && group_hash != kMainGroupHash) {
    BaseView::GetSharedEngine()->destroy(group.scene);
    view_->GetDispatcher().Send(GroupDestroyedEvent(GetGroupName(group_hash)));
    groups_.erase(group_hash);
    hashes_to_names_.erase(group_hash);
  }
}

void GroupsManager::SetNodeActiveInGroup(HashValue group_hash, NodeHandle node,
                                         bool is_active) {
  auto itr = groups_.find(group_hash);
  if (itr == groups_.end()) {
    IMP_LOG(imp::FATAL) << "Attempting to set node active in a group that doesn't "
                  "exist.";
    return;
  }

  Group& group = itr->second;
  if (is_active) {
    group.scene->addEntity(node.GetEntity());
  } else {
    group.scene->remove(node.GetEntity());
  }
}

filament::Scene* GroupsManager::GetScene(absl::string_view group_name) {
  HashValue group_hash = Hash(group_name);
  auto itr = groups_.find(group_hash);
  if (itr == groups_.end()) {
    return nullptr;
  }
  Group& group = itr->second;
  return group.scene;
}

size_t GroupsManager::GetNumNodesInGroup(absl::string_view group_name) {
  HashValue group_hash = Hash(group_name);
  auto itr = groups_.find(group_hash);
  if (itr == groups_.end()) {
    return 0;
  }
  Group& group = itr->second;
  return group.num_nodes_in_layer;
}

absl::string_view GroupsManager::GetGroupName(HashValue group_hash) {
  auto itr = hashes_to_names_.find(group_hash);
  if (itr == hashes_to_names_.end()) {
    return {};
  }

  return itr->second;
}

}  // namespace imp
