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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_ATTACHMENT_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_ATTACHMENT_MANAGER_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/entity_absl_hasher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_controller.h"
#include "core/ncsb/node_flag.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::imp_internal {

// NodeAttachmentManager is used by the View to manage the nodes in the View's
// scene graph.
//
// In Impress, each Node is a filament::Entity that has a NodeController
// associated with it and a filament transform. This class is responsible for
// creating the NodeControllers and managing their association with
// filament::Entities.
//
// It also allows static lookup of a NodeController from a filament::Entity.
// This makes it possible to construct a NodeHandle from an Entity without
// access to the View.
//
// Each filament::Entity can only be associated with one View.
class NodeAttachmentManager {
 public:
  explicit NodeAttachmentManager(BaseView* view) noexcept;
  ~NodeAttachmentManager();

  // Get the NodeController associated with the entity passed in.
  //
  // Returns nullptr if the entity is not associated with a NodeController.
  static NodeController* Get(utils::Entity entity);

  // If the entity is already associated with a NodeController, then return it.
  //
  // Otherwise, convert this entity into a node and return it.
  //
  // This is done by creating a NodeController and associating it with the
  // entity as well as ensuring it has a transform.
  //
  // Note: This takes a pre-existing entity instead of creating one
  // intentionally, because it must be possible to convert filament entities
  // created by other systems into nodes.
  NodeHandle Attach(utils::Entity entity);

  // Destroys the node passed in.
  //
  // Destroys both the NodeController and the corresponding entity it is
  // associated with.
  void Destroy(NodeHandle node) noexcept;

  // Returns the total number of NodeControllers associated with this manager.
  std::size_t GetCount() const;

  // Destroys all NodeControllers and their corresponding entities.
  // This is done during shutdown of the View only.
  void Cleanup();

  // Iterate over all nodes in this manager.
  //
  // The function passed in must be callable as:
  //   void fn(NodeHandle node)
  template <typename Fn>
  void ForEach(Fn&& fn);

  // Iterate over a subset of all Nodes in this manager. Only nodes that meet
  // all flags will be included. i.e. ForEach(fn, NodeFlags::kIsEnabled |
  // NodeFlags::kIsRoot) will only include enabled root nodes.
  //
  // The function passed in must be callable as:
  //   void fn(NodeHandle node)
  template <typename Fn>
  void ForEach(Fn&& fn, NodeFlag filter);

 private:
  using EntitiesToControllersMap =
      absl::flat_hash_map<utils::Entity, NodeController*, EntityHasher>;

  static EntitiesToControllersMap& GetEntitiesToControllersMap();

  BaseView* view_;
  std::vector<std::unique_ptr<NodeController>> node_controllers_;
};

template <typename Fn>
void NodeAttachmentManager::ForEach(Fn&& fn) {
  // Iterate using indices so that it is safe to create/destroy nodes during
  // iteration.
  // TODO: Ensure it ForEachNode doesn't iterate over the same node
  // multiple times or skip nodes when destroying during iteration.
  for (std::size_t i = 0; i < node_controllers_.size(); ++i) {
    fn(node_controllers_[i]->GetNode());
  }
}

template <typename Fn>
void NodeAttachmentManager::ForEach(Fn&& fn, NodeFlag filter) {
  // Iterate using indices so that it is safe to create/destroy nodes during
  // iteration.
  // TODO: Ensure it ForEachNode doesn't iterate over the same node
  // multiple times or skip nodes when destroying during iteration.
  for (std::size_t i = 0; i < node_controllers_.size(); ++i) {
    NodeController* node_controller = node_controllers_[i].get();
    if ((node_controller->GetFlags() & filter) == filter) {
      fn(node_controller->GetNode());
    }
  }
}

}  // namespace imp::imp_internal

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_ATTACHMENT_MANAGER_H_
