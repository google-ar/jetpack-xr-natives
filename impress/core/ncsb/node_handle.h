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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_HANDLE_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_HANDLE_H_

#include <functional>
#include <string>

#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/hash.h"
#include "core/ncsb/base_node.h"

namespace imp {

class Node;

namespace imp_internal {
class NodeAttachmentManager;
}  // namespace imp_internal

// Provides access to a Node with semantics that behave similarly to a weak
// pointer. The NodeHandle remains valid until View::DestroyNode is called for
// this node or this node's ancestor.
//
// NodeHandle is cheap to copy and should be passed by value.
class NodeHandle {
 public:
  // Creates an invalid NodeHandle.
  NodeHandle();

  // Creates a NodeHandle for the entity passed in. The NodeHandle will be
  // invalid if the entity is invalid or if the entity isn't attached to the
  // View.
  explicit NodeHandle(utils::Entity entity);

  // Creates a NodeHandle for the already existing Node.
  explicit NodeHandle(const Node& node);

  // Accesses the Node as a reference. Asserts that the Node is valid.
  Node& operator*() const noexcept;

  // Accesses the Node as a pointer. Asserts that the Node is valid.
  Node* operator->() const noexcept;

  // Checks if two NodeHandles reference the same Node.
  bool operator==(const NodeHandle& other) const;
  bool operator!=(const NodeHandle& other) const;

  // Returns true if the NodeHandle references a valid node.
  explicit operator bool() const noexcept;

  // Returns true if the NodeHandle references a valid node.
  bool IsValid() const;

  // Returns true if the NodeHandle does not reference a node, such as created
  // from the default constructor. Previously valid but then destroyed nodes
  // will be invalid, but still not this value.
  // IMPORTANT! Please do NOT use this unless you care about the exact value of
  // the NodeHandle, such as keying a hashmap or if a node was ever created
  // before. Prefer to use IsValid() or operator bool() instead, as they take
  // into account the Node's destroyed state. NodeHandles can be not null, but
  // destroyed, and will therefore still assert if you dereference it.
  bool IsDefaultValue() const;

  // Returns the filament entity that this NodeHandle is wrapping.
  // Do not use this API unless you understand the underlying details of
  // filament.
  utils::Entity GetEntity() const;

  // These kTypeUrl fields allow NodeHandle to be used directly as a proto.
  // This works in conjunction with NodeHandleMessage.
  static constexpr const char* kTypeUrl =
      "type.googleapis.com/imp.NodeHandleMessage";
  static constexpr imp::HashValue kTypeUrlHash =
      imp::ConstHash("type.googleapis.com/imp.NodeHandleMessage");

  template <typename H>
  friend H AbslHashValue(H hash, const NodeHandle& handle) {
    return H::combine(std::move(hash), handle.node_.GetEntity().getId());
  }

  template <typename Sink>
  friend void AbslStringify(Sink& sink, const NodeHandle& node);

 private:
  NodeHandle(utils::Entity entity,
             imp_internal::NodeController* node_controller);

  friend struct std::hash<NodeHandle>;

  void AssertIsValid() const;

  // Marking node_ mutable because a const NodeHandle should not be a handle to
  // a constant node - instead it's a constant handle to a mutable node (meaning
  // that the node the const NodeHandle references can't change, but the node
  // can be mutated).
  mutable BaseNode node_;

  friend class imp_internal::NodeController;
  friend class imp_internal::NodeAttachmentManager;
};

std::string ToString(const NodeHandle& handle);

template <typename Sink>
void AbslStringify(Sink& sink, const NodeHandle& node) {
  sink.Append(ToString(node));
}
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_HANDLE_H_
