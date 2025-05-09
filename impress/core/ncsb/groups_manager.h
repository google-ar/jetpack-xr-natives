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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_GROUPS_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_GROUPS_MANAGER_H_

#include <cstddef>
#include <functional>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Scene.h"
#include "core/common/hash.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp {

namespace imp_internal {
class NodeController;
}

// Stores information about all of the  groups that currently exist.
//
// Groups are created and destroyed dynamically (except for the main
// group). When all nodes are removed from a group, the group is destroyed. If a
// node is then added to that group in the future it will be recreated.
//
// Internally, when a node is active in a group it is added to a corresponding
// filament::Scene for rendering.
class GroupsManager {
 public:
  // GroupCreatedEvent will be sent out through imp::View's Dispatcher when a
  // group is created.
  //
  // Please note that GroupCreatedEvent for main group is omitted as main group
  // is guaranteed to always exist.
  struct GroupCreatedEvent : public Event {
    GroupCreatedEvent(absl::string_view group_name) : group_name(group_name) {}

    std::string group_name;
  };

  // GroupCreatedEvent will be sent out through imp::View's Dispatcher when a
  // group is destroyed.
  struct GroupDestroyedEvent : public Event {
    GroupDestroyedEvent(absl::string_view group_name)
        : group_name(group_name) {}

    std::string group_name;
  };

  static constexpr absl::string_view kMainGroupName = "Main";
  static constexpr HashValue kMainGroupHash = Hash(kMainGroupName);

  explicit GroupsManager(BaseView* view, filament::Scene* main_scene);

  // Used to iterate over each node that is currently active and within the
  // specified group.
  template <typename Fn>
  void ForEachActiveNodeInGroup(absl::string_view group_name, Fn fn);

  // Returns the filament::Scene containing all of the active nodes within the
  // specified group.
  //
  // This is used to render the group using filament.
  //
  // The filament::Scene will be destroyed if all nodes are removed from the
  // scene (except for the main group), so it isn't safe to indefinitely hold
  // onto the returned scene.
  filament::Scene* GetScene(absl::string_view group_name);

  // Returns the number of nodes that are currently part of the specified group.
  size_t GetNumNodesInGroup(absl::string_view group_name);

  // Returns the name of a group given the hash of that groups name.
  // If there is no node in the group, then returns an empty string_view.
  absl::string_view GetGroupName(HashValue group_hash);

 private:
  struct Group {
    filament::Scene* scene;
    size_t num_nodes_in_layer = 0;
  };

  void AddNodeToGroup(absl::string_view group_name, HashValue group_hash,
                      NodeHandle node);

  void RemoveNodeFromGroup(HashValue group_hash, NodeHandle node);

  void SetNodeActiveInGroup(HashValue group_hash, NodeHandle node,
                            bool is_active);

  // NodeController needs to call AddNodeToGroup, RemoveNodeFromGroup, and
  // SetNodeActiveInGroup which is used to implement the public API on Node for
  // controlling groups.
  friend class imp_internal::NodeController;

  BaseView* view_;

  absl::flat_hash_map<HashValue, Group> groups_;
  absl::flat_hash_map<HashValue, std::string> hashes_to_names_;
};

template <typename Fn>
void GroupsManager::ForEachActiveNodeInGroup(absl::string_view group_name,
                                             Fn fn) {
  auto itr = groups_.find(Hash(group_name));
  if (itr == groups_.end()) {
    return;
  }
  Group& groups = itr->second;

  // Instead of iterating through every node and checking if it's in the group,
  // it would be better to just iterate through the entities in the
  // filament::Scene. However, filament::Scene doesn't provide a way to do that
  // today. We could also track the nodes ourselves, but that is extra
  // unnecessary memory. Filament is currently working on adding an API for
  // this.
  // TODO: Use the new filament API when it becomes available.
  view_->ForEachNode([&groups, &fn](NodeHandle node) {
    if (groups.scene->hasEntity(node.GetEntity())) {
      fn(node);
    }
  });
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_GROUPS_MANAGER_H_
