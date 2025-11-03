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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_BASE_NODE_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_BASE_NODE_H_

#include <iterator>

#include "filament/libs/utils/include/utils/Entity.h"
#include "core/ncsb/component_handle.h"

namespace imp {

class NodeHandle;

namespace imp_internal {
class NodeController;
class NodeAttachmentManager;
}  // namespace imp_internal

// A base class held by NodeHandle that can static_cast to Node in
// node_handle.cc, while node_handle.h has no dependency on node.h. This means
// features can depend on public node_handle.h API, while node.h can use those
// same features without a circular dependency.
// TODO: Resolve the circular dependency between node.h and
// node_handle.h and remove this class.
class BaseNode {
 public:
  // Checks to see if the Node represents the same entity.
  bool operator==(const BaseNode& other) const;
  bool operator!=(const BaseNode& other) const;

  // Returns the filament entity that this Node is wrapping.
  // Do not use this API unless you understand the underlying details of
  // filament.
  inline utils::Entity GetEntity() const { return entity_; }

 protected:
  explicit BaseNode(utils::Entity entity);
  BaseNode(utils::Entity entity, imp_internal::NodeController* node_controller);

  // The filament entity that this node is wrapping. The entity id has a 1:1
  // mapping with the node & is used to look up the components associated with
  // the node.
  utils::Entity entity_;

  // Each node is a filament Entity that has a NodeController associated with
  // it. This is the pointer to the NodeController for this node.
  //
  // The NodeController is used for implementing functionality like
  // enabled/active, groups, name, etc.
  imp_internal::NodeController* node_controller_ = nullptr;

  friend class NodeHandle;

  // Allows NodeController and NodeAttachmentManager to access the
  // node_controller_ field directly.
  friend class imp_internal::NodeController;
  friend class imp_internal::NodeAttachmentManager;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_BASE_NODE_H_
