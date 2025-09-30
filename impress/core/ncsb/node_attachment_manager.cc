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

#include "core/ncsb/node_attachment_manager.h"

#include <cstddef>
#include <memory>
#include <utility>

#include "absl/log/check.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/EntityManager.h"
#include "core/common/vector_helpers.h"
#include "core/ncsb/node_controller.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"

namespace imp::imp_internal {

NodeAttachmentManager::NodeAttachmentManager(BaseView* view) noexcept
    : view_(view) {}

NodeAttachmentManager::~NodeAttachmentManager() {
  // All nodes should be destroyed before the NodeAttachmentManager is
  // destroyed.
  
}

NodeAttachmentManager::EntitiesToControllersMap&
NodeAttachmentManager::GetEntitiesToControllersMap() {
  thread_local EntitiesToControllersMap entities_to_controllers;
  return entities_to_controllers;
}

NodeController* NodeAttachmentManager::Get(utils::Entity entity) {
  EntitiesToControllersMap& entities_to_controllers =
      GetEntitiesToControllersMap();
  auto itr = entities_to_controllers.find(entity);
  if (itr != entities_to_controllers.end()) {
    return itr->second;
  } else {
    return nullptr;
  }
}

NodeHandle NodeAttachmentManager::Attach(utils::Entity entity) {
  if (!entity) {
    return {};
  }

  // If the entity is already associated with a NodeController, then return it.
  EntitiesToControllersMap& entities_to_controllers =
      GetEntitiesToControllersMap();
  auto itr = entities_to_controllers.find(entity);
  if (itr != entities_to_controllers.end()) {
    
    return NodeHandle(entity, itr->second);
  }

  // Ensure that the entity has a transform.
  filament::TransformManager& tm =
      view_->GetSharedEngine()->getTransformManager();
  if (!tm.hasComponent(entity)) {
    tm.create(entity);
  }

  auto node_controller =
      std::make_unique<NodeController>(view_, entity, node_controllers_.size());
  NodeController* node_controller_ptr = node_controller.get();
  node_controllers_.push_back(std::move(node_controller));

  // Track the association between the entity and the node controller.
  entities_to_controllers.emplace(entity, node_controller_ptr);

  // Call PostCreated. This work isn't done in the constructor because it
  // requires the NodeController to already be in the NodeAttachmentManager.
  node_controller_ptr->PostCreated();

  if (split_engine::SplitEngineSerializer* serializer =
          view_->GetSplitEngineSerializer()) {
    serializer->CreateNode(entity);
  }

  return NodeHandle(entity, node_controller_ptr);
}

void NodeAttachmentManager::Destroy(NodeHandle node) noexcept {
  if (!node) {
    // Do nothing if the node is already destroyed or empty.
    //
    // This can happen when View::DestroyNode is called if the node gets
    // destroyed by removing a component.
    return;
  }

  NodeController* node_controller = node->node_controller_;
  std::size_t index = node_controller->GetIndex();
  
  

  // First call PreDestroyed, which must be done prior to removing the node
  // from the NodeAttachmentManager & node_controllers_ vector, which is why
  // this work is done here instead of in the destructor.
  node_controller->PreDestroyed();

  // Remembered objects are destroyed in the NodeController::PreDestroyed.
  // It's possible that the node was destroyed when ClearRemembered was running.
  // In that case, we don't need to do anything so can return early here.
  if (!node) {
    return;
  }

  // It's possible that the index changed while PreDestroyed was running if a
  // different node was destroyed.
  index = node_controller->GetIndex();

  utils::Entity entity = node_controller->GetEntity();

  EntitiesToControllersMap& entities_to_controllers =
      GetEntitiesToControllersMap();
  entities_to_controllers.erase(entity);

  // Special case for removing a node while in the midst of iterating over
  // the nodes. In this case, we can't swap and pop because it can cause
  // the iteration to skip over nodes. Instead we leave a gap in the vector and
  // compact it after iteration ends.
  if (iterating_depth_ > 0) {
    node_controllers_[index].reset();
    needs_compaction_ = true;
  } else {
    std::size_t last_index = node_controllers_.size() - 1;
    // To prevent the node_controllers_ vector from having gaps, we swap the
    // node_controllers_ the node to the last index, and then remove the last
    // index.
    if (index != last_index) {
      std::swap(node_controllers_[index], node_controllers_[last_index]);
      // Update the index of the node that was swapped in.
      node_controllers_[index]->SetIndex(index);
    }
    node_controllers_.pop_back();
  }

  // Ensure filament components are removed from the entity before we destroy
  // it. If we don't do this, filament should eventually clean up the components
  // attached to a dead entity anyways, but better for us to explicitly remove
  // them first.
  view_->GetSharedEngine()->destroy(entity);

  // Destroys the actual entity in filament's entity manager.
  utils::EntityManager::get().destroy(entity);
}

std::size_t NodeAttachmentManager::GetCount() const {
  return node_controllers_.size();
}

void NodeAttachmentManager::Cleanup() {
  EntitiesToControllersMap& entities_to_controllers =
      GetEntitiesToControllersMap();
  utils::EntityManager& em = utils::EntityManager::get();
  filament::Engine* engine = BaseView::GetSharedEngine();

  for (std::unique_ptr<imp_internal::NodeController>& node_controller :
       node_controllers_) {
    // Must be called before the node controller is removed from the lookup.
    node_controller->PreDestroyed();

    // Disassociate the node controller from the entity.
    entities_to_controllers.erase(node_controller->GetEntity());

    utils::Entity entity = node_controller->GetEntity();
    if (em.isAlive(entity)) {
      // This is a saving-throw to remove any filament components (i.e.
      // renderable) from the entity before we destroy it. If we don't do
      // this, filament should eventually clean up the components attached to
      // a dead entity anyways, but better for us to explicitly remove them
      // first.
      engine->destroy(entity);

      // Destroys the actual entity in filament's entity manager.
      em.destroy(entity);
    }
  }

  // Destroy all node controllers.
  node_controllers_.clear();
}

void NodeAttachmentManager::TryCompactingNodeControllers() {
  if (iterating_depth_ > 0) {
    // If we are iterating, don't compact the vector.
    return;
  }

  if (!needs_compaction_) {
    // No need to compact the vector.
    return;
  }

  CompactVector(node_controllers_, [this](size_t new_index) {
    // When a node controller's index is changed within the vector, the index
    // the controller is associated with must be updated.
    node_controllers_[new_index]->SetIndex(new_index);
  });

  needs_compaction_ = false;
}

}  // namespace imp::imp_internal
